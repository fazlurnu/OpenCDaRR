"""Handbook figures: fixed-wing coordinated-turn trajectories.

Drives the real :class:`~opencdarr.kinematics.FixedWing` (SMALL_FIXEDWING): a speed-dependent
banked turn (with the bank angle over time) and three position modes (pure-pursuit go-to, L1 leg
tracking, loiter orbit). Writes into the site repo. Plot style: no grid, no suptitle, square
subplots.

    PYTHONPATH=. python scripts/handbook/kinematics_fixedwing.py
"""
from __future__ import annotations

import math
from collections.abc import Callable
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.kinematics import FixedWing, MotionCommand  # noqa: E402
from opencdarr.performance import SMALL_FIXEDWING as FW  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

DT = 0.5
LAT0, LON0 = 52.0, 4.0
KINEMATICS = FixedWing()
IMG = Path.home() / "Projects/opencdarr.github.io/docs/assets/img"

CommandFn = Callable[[AircraftState, float], MotionCommand]


def fw_state(v: float, yaw: float = 0.0, at: tuple[float, float] | None = None) -> AircraftState:
    lat, lon = at if at is not None else (LAT0, LON0)
    return AircraftState(id="FW", lat=lat, lon=lon, trk=yaw, gs=v, yaw=yaw, bank=0.0)


def run(s0: AircraftState, fn: CommandFn, t_max: float) -> list[AircraftState]:
    states, s, t = [s0], s0, 0.0
    while t < t_max:
        s = KINEMATICS.step(s, fn(s, t), FW, DT)
        states.append(s)
        t += DT
    return states


def enu(point: tuple[float, float]) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(LAT0, LON0, point[0], point[1])
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def track(states: list[AircraftState]) -> tuple[list[float], list[float]]:
    pts = [enu((s.lat, s.lon)) for s in states]
    return [p[0] for p in pts], [p[1] for p in pts]


def square(ax: plt.Axes, xs: list[float], ys: list[float], min_half: float = 20.0) -> None:
    """Equal, square data limits containing (xs, ys), so the axes box is square."""
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    half = max(max(xs) - min(xs), max(ys) - min(ys)) / 2
    half = max(half, min_half) * 1.12
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_aspect("equal")


# ------------------------------------------------------------------ figure 1: banked turn
def turn_figure() -> None:
    fig, (ax_xy, ax_phi) = plt.subplots(1, 2, figsize=(11.0, 5.2))

    def course(_s: AircraftState, t: float) -> MotionCommand:  # north, then east at t=5 s
        return MotionCommand(target_course=0.0 if t < 5.0 else 90.0, target_airspeed=v)

    all_x, all_y = [], []
    for v, col in ((15.0, "#1f77b4"), (25.0, "#ff7f0e")):
        states = run(fw_state(v), course, 22.0)
        xs, ys = track(states)
        all_x += xs
        all_y += ys
        ts = [i * DT for i in range(len(states))]
        ax_xy.plot(xs, ys, color=col, lw=2.0, label=f"{v:.0f} m/s")
        ax_xy.plot(xs[0], ys[0], "^", color=col, ms=8)
        ax_phi.plot(ts, [s.bank for s in states], color=col, lw=2.0, label=f"{v:.0f} m/s")

    ax_xy.set_title("ground track", fontsize=10)
    ax_xy.set_xlabel("east [m]")
    ax_xy.set_ylabel("north [m]")
    square(ax_xy, all_x, all_y)
    ax_xy.legend(fontsize=9)

    ax_phi.axhline(FW.phi_max, color="0.5", ls="--", lw=1.0)
    ax_phi.set_title("bank angle", fontsize=10)
    ax_phi.set_xlabel("time [s]")
    ax_phi.set_ylabel("bank φ [deg]")
    ax_phi.set_box_aspect(1)
    ax_phi.legend(fontsize=9)

    fig.tight_layout()
    fig.savefig(IMG / "fw-turn.png", dpi=130)
    print("wrote", IMG / "fw-turn.png")


# ------------------------------------------------------------------ figure 2: position modes
def paths_figure() -> None:
    fig, (ax_go, ax_wp, ax_lo) = plt.subplots(1, 3, figsize=(15.0, 5.2))
    a, b = (LAT0, LON0), geo.forward(LAT0, LON0, 0.0, 300.0)
    start = geo.forward(LAT0, LON0, 90.0, 80.0)
    be, bn = enu(b)

    # (1) pure pursuit: bare target_position, no leg — steers straight at the point
    go = run(fw_state(18.0, yaw=0.0, at=start),
             lambda s, t: MotionCommand(target_position=b, target_airspeed=18.0), 18.0)
    xs, ys = track(go)
    ax_go.plot(xs, ys, color="#d62728", lw=2.0, label="path")
    ax_go.plot(xs[0], ys[0], "^", color="#d62728", ms=8)
    ax_go.plot(be, bn, "*", color="0.3", ms=13, label="target")
    ax_go.set_title("go-to (pursuit)", fontsize=10)
    square(ax_go, xs + [be], ys + [bn])
    ax_go.legend(fontsize=9)

    # (2) L1 waypoint: leg A→B, start 80 m off the leg — curves onto the line
    wp = run(fw_state(18.0, yaw=0.0, at=start),
             lambda s, t: MotionCommand(target_position=b, target_leg_start=a, target_airspeed=18.0),
             30.0)
    ae, an = enu(a)
    xs, ys = track(wp)
    ax_wp.plot([ae, be], [an, bn], color="0.6", ls="--", lw=1.2, label="leg A→B")
    ax_wp.plot(xs, ys, color="#1f77b4", lw=2.0, label="path")
    ax_wp.plot(xs[0], ys[0], "^", color="#1f77b4", ms=8)
    ax_wp.set_title("L1 leg tracking", fontsize=10)
    square(ax_wp, xs + [ae, be], ys + [an, bn])
    ax_wp.legend(fontsize=9)

    # (3) loiter: orbit the origin at radius 60 m
    radius = 60.0
    lo = run(fw_state(18.0, yaw=0.0, at=geo.forward(LAT0, LON0, 90.0, 130.0)),
             lambda s, t: MotionCommand(target_position=(LAT0, LON0), target_loiter_radius=radius,
                                        target_airspeed=18.0), 45.0)
    xs, ys = track(lo)
    ax_lo.add_patch(plt.Circle((0.0, 0.0), radius, color="0.6", ls="--", lw=1.2, fill=False,
                               label=f"radius {radius:.0f} m"))
    ax_lo.plot(xs, ys, color="#2ca02c", lw=2.0, label="path")
    ax_lo.plot(xs[0], ys[0], "^", color="#2ca02c", ms=8)
    ax_lo.set_title("loiter orbit", fontsize=10)
    square(ax_lo, xs + [-radius, radius], ys + [-radius, radius])
    ax_lo.legend(fontsize=9)

    for ax in (ax_go, ax_wp, ax_lo):
        ax.set_xlabel("east [m]")
        ax.set_ylabel("north [m]")

    fig.tight_layout()
    fig.savefig(IMG / "fw-paths.png", dpi=130)
    print("wrote", IMG / "fw-paths.png")


def main() -> None:
    IMG.mkdir(parents=True, exist_ok=True)
    turn_figure()
    paths_figure()


if __name__ == "__main__":
    main()
