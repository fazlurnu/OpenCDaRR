"""Handbook figures: the multirotor MotionCommand gallery.

For every cell of the translation x yaw grid (and the edge cases), drive the real
:class:`~opencdarr.dynamics.Multirotor` (M600) and plot the ground track with nose arrows, so
facing versus travel is visible. Writes one figure per group into the site repo.

    PYTHONPATH=. python scripts/handbook/dynamics_multirotor_gallery.py
"""
from __future__ import annotations

import math
from collections.abc import Callable
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.dynamics import Multirotor, MotionCommand  # noqa: E402
from opencdarr.performance import M600  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

DT = 0.5
LAT0, LON0 = 52.0, 4.0
DYN = Multirotor()
IMG = Path.home() / "Projects/opencdarr.github.io/docs/assets/img"
TRACK, NOSE = "#1f77b4", "#888888"

CommandFn = Callable[[AircraftState, float], MotionCommand]


def hover(yaw: float | None = None) -> AircraftState:
    return AircraftState(id="M600", lat=LAT0, lon=LON0, trk=0.0, gs=0.0, yaw=yaw)


def cruise(trk: float, gs: float) -> AircraftState:
    return AircraftState(id="M600", lat=LAT0, lon=LON0, trk=trk, gs=gs)


def run(s0: AircraftState, fn: CommandFn, t_max: float) -> list[AircraftState]:
    states = [s0]
    s, t = s0, 0.0
    while t < t_max:
        s = DYN.step(s, fn(s, t), M600, DT)
        states.append(s)
        t += DT
    return states


def enu(s: AircraftState) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(LAT0, LON0, s.lat, s.lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


_ARROW = 14.0  # nose-arrow length [m]


def plot_case(ax: plt.Axes, states: list[AircraftState], title: str) -> None:
    """Ground track plus nose arrows (yaw, or track when yaw is None). No grid, concise title."""
    pts = [enu(s) for s in states]
    txs, tys = [p[0] for p in pts], [p[1] for p in pts]
    ax.plot(txs, tys, color=TRACK, lw=2.0, zorder=2)
    ax.plot(txs[0], tys[0], "^", color=TRACK, ms=8, zorder=4)
    lim_x, lim_y = list(txs), list(tys)  # points the axes must contain (track + arrow tips)
    for k in range(0, len(states), 3):  # a nose arrow every 1.5 s
        head = states[k].yaw if states[k].yaw is not None else states[k].trk
        r = math.radians(head)
        tip = (pts[k][0] + math.sin(r) * _ARROW, pts[k][1] + math.cos(r) * _ARROW)
        ax.annotate("", xy=tip, xytext=pts[k],
                    arrowprops=dict(arrowstyle="->", color=NOSE, lw=1.3))
        lim_x.append(tip[0])
        lim_y.append(tip[1])
    # square limits containing track + arrow tips, with a floor so nose-aligned cases don't collapse
    cx, cy = (min(lim_x) + max(lim_x)) / 2, (min(lim_y) + max(lim_y)) / 2
    half = max(max(lim_x) - min(lim_x), max(lim_y) - min(lim_y)) / 2
    half = max(half, 10.0) * 1.15
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("east [m]")
    ax.set_ylabel("north [m]")
    ax.set_aspect("equal")


def yaw_row(fname: str, s0_fn: Callable[[], AircraftState], base: Callable[[float | None, float | None], MotionCommand], t_max: float) -> None:
    """A 1x3 row: the same translation command under hold / target_yaw / target_yawspeed."""
    fig, axs = plt.subplots(1, 3, figsize=(12.5, 4.4))
    plot_case(axs[0], run(s0_fn(), lambda s, t: base(None, None), t_max), "hold")
    plot_case(axs[1], run(s0_fn(), lambda s, t: base(90.0, None), t_max), "target_yaw 90°")
    plot_case(axs[2], run(s0_fn(), lambda s, t: base(None, 45.0), t_max), "target_yawspeed 45°/s")
    fig.tight_layout()
    fig.savefig(IMG / fname, dpi=130)
    print("wrote", IMG / fname)


def main() -> None:
    IMG.mkdir(parents=True, exist_ok=True)

    # 1. inertial target_velocity, north at 15 m/s
    yaw_row(
        "mc-velocity.png", lambda: hover(),
        lambda ty, tys: MotionCommand(target_velocity=(0.0, 15.0), target_yaw=ty, target_yawspeed=tys),
        8.0,
    )

    # 2. body target_body_velocity, forward 15 m/s (nose starts north)
    yaw_row(
        "mc-body.png", lambda: hover(0.0),
        lambda ty, tys: MotionCommand(target_body_velocity=(15.0, 0.0), target_yaw=ty, target_yawspeed=tys),
        12.0,
    )

    # 3. target_position, a point 130 m out on bearing 040
    goto = geo.forward(LAT0, LON0, 40.0, 130.0)
    yaw_row(
        "mc-position.png", lambda: hover(),
        lambda ty, tys: MotionCommand(target_position=goto, target_yaw=ty, target_yawspeed=tys),
        16.0,
    )

    # 4. edge cases: priority (position beats velocity) and hover (zero velocity from cruise)
    fig, axs = plt.subplots(1, 2, figsize=(8.6, 4.4))
    plot_case(
        axs[0],
        run(hover(), lambda s, t: MotionCommand(target_position=goto, target_velocity=(15.0, 0.0)), 16.0),
        "position > velocity",
    )
    plot_case(
        axs[1],
        run(cruise(0.0, 15.0), lambda s, t: MotionCommand.from_velocity(0.0, 0.0), 6.0),
        "zero velocity → hover",
    )
    fig.tight_layout()
    fig.savefig(IMG / "mc-edge.png", dpi=130)
    print("wrote", IMG / "mc-edge.png")


if __name__ == "__main__":
    main()
