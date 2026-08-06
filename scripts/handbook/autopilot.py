"""Handbook figures: the autopilot (guidance / navigator) layer.

The "Autopilot" module page's pictures, drawn from the real
:class:`~opencdarr.autopilot.WaypointAutopilot` driving the real airframes:

  1. ``mission`` — one flight plan, flown by a multirotor and a fixed-wing through the same
     ``autopilot.step -> kinematics.step`` loop. Because the autopilot emits a *position* setpoint,
     one navigator serves both airframes: the multirotor flies straight in and hovers at the final
     waypoint; the fixed-wing rounds the corners with L1 leg tracking and orbits the last one.
  2. ``l1`` — the L1 guidance law the emitted leg invokes: the construction (foot of the
     perpendicular, the L1 reference point, the commanded course) computed from the real formulas,
     and the cross-track capture of a leg from either side.

Handbook plot style: no suptitle, concise titles, no grid. Writes into the site repo.

    PYTHONPATH=. python scripts/handbook/autopilot.py
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.autopilot import GuidanceMemory, WaypointAutopilot  # noqa: E402
from opencdarr.kinematics import FixedWing, MotionCommand, Multirotor  # noqa: E402
from opencdarr.kinematics.base import Kinematics  # noqa: E402
from opencdarr.kinematics.fixedwing import _L1_DISTANCE  # noqa: E402
from opencdarr.mission import Mission, Waypoint  # noqa: E402
from opencdarr.performance import M600, SMALL_FIXEDWING, Performance  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

LAT0, LON0 = 52.0, 4.0
IMG = Path.home() / "Projects/opencdarr.github.io/docs/assets/img"
BLUE, ORANGE, GREY, RED, GREEN = "#1f77b4", "#ff7f0e", "0.55", "#d62728", "#2ca02c"
_MR, _FW = Multirotor(), FixedWing()


def _enu(lat: float, lon: float) -> tuple[float, float]:
    """(east, north) [m] of a lat/lon relative to the origin."""
    qdr, dist = geo.qdrdist(LAT0, LON0, lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def _latlon(e: float, n: float) -> tuple[float, float]:
    """The (lat, lon) of an ENU (east, north) offset from the origin."""
    return geo.forward(LAT0, LON0, math.degrees(math.atan2(e, n)) % 360.0, math.hypot(e, n))


# ----------------------------------------------------------- figure 1: mission, two airframes
# A flight plan in ENU metres from the origin: a couple of turns, then a final loiter waypoint.
PLAN_ENU: list[tuple[float, float]] = [
    (0.0, 260.0), (330.0, 400.0), (150.0, 660.0), (470.0, 780.0),
]
CAPTURE, LOITER, CRUISE = 40.0, 80.0, 17.0


def _fly_mission(kinematics: Kinematics, perf: Performance, s0: AircraftState,
                 tmax: float, dt: float = 0.2) -> tuple[list[float], list[float]]:
    """Fly ``PLAN_ENU`` through the real ``WaypointAutopilot`` -> ``Kinematics`` loop (no wind),
    exactly as the fleet runner threads the two layers. Returns the ground track (east, north)."""
    plan = tuple(Waypoint(*_latlon(e, n)) for e, n in PLAN_ENU)
    ap = WaypointAutopilot(Mission(flight_plan=plan), cruise_airspeed=CRUISE,
                           capture_radius=CAPTURE, loiter_radius=LOITER)
    gm = GuidanceMemory()
    s, t = s0, 0.0
    xs, ys = [], []
    while t < tmax:
        e, n = _enu(s.lat, s.lon)
        xs.append(e)
        ys.append(n)
        cmd, gm = ap.step(s, gm, perf)
        s = kinematics.step(s, cmd, perf, dt)
        t += dt
    return xs, ys


def _draw_plan(ax: plt.Axes, loiter: bool) -> None:
    """The plan geometry: legs, waypoints, capture rings, and (fixed-wing only) the loiter ring at
    the final waypoint. A multirotor hovers *on* the final point, so it gets no loiter ring."""
    es = [0.0] + [e for e, _ in PLAN_ENU]
    ns = [0.0] + [n for _, n in PLAN_ENU]
    ax.plot(es, ns, color=GREY, ls="--", lw=1.2, zorder=1)
    for i, (e, n) in enumerate(PLAN_ENU):
        final = i == len(PLAN_ENU) - 1
        ax.plot(e, n, "*", color="0.2", ms=15, zorder=4)
        if final and loiter:
            ax.add_patch(plt.Circle((e, n), LOITER, fill=False, ls=":", lw=1.0, color=RED,
                                    alpha=0.8, zorder=1))
        elif not final:
            ax.add_patch(plt.Circle((e, n), CAPTURE, fill=False, ls=":", lw=1.0, color="0.6",
                                    alpha=0.8, zorder=1))


def mission_figure(out: Path) -> None:
    mr = AircraftState(id="MR", lat=LAT0, lon=LON0, trk=0.0, gs=10.0)
    fw = AircraftState(id="FW", lat=LAT0, lon=LON0, trk=0.0, gs=CRUISE, yaw=0.0, bank=0.0)
    mr_xy = _fly_mission(_MR, M600, mr, tmax=150.0)
    fw_xy = _fly_mission(_FW, SMALL_FIXEDWING, fw, tmax=150.0)

    fig, (a_mr, a_fw) = plt.subplots(1, 2, figsize=(11.5, 6.2), sharex=True, sharey=True)
    for ax, (xs, ys), col, title, loiter in (
        (a_mr, mr_xy, BLUE, "Multirotor: fly straight in, hover", False),
        (a_fw, fw_xy, ORANGE, "Fixed-wing: L1 leg tracking, loiter", True),
    ):
        _draw_plan(ax, loiter)
        ax.plot(xs, ys, color=col, lw=2.2, zorder=3)
        ax.plot(xs[0], ys[0], "^", color=col, ms=9, zorder=4)
        ax.plot(xs[-1], ys[-1], "o", color=col, ms=7, zorder=4)
        ax.set_aspect("equal")
        ax.set_xlabel("east [m]")
        ax.set_title(title, fontsize=10)
    a_mr.set_ylabel("north [m]")
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    print("wrote", out)


# ------------------------------------------------------------------ figure 2: L1 leg tracking
def _l1_reference(px: float, py: float, ax_: float, ay: float, bx: float,
                  by: float) -> tuple[tuple[float, float], tuple[float, float], float]:
    """The L1 construction for an aircraft at ``P`` and a leg ``A -> B`` (the derivation, world
    ENU frame): the foot ``F`` of the perpendicular, the reference point ``R`` on the leg, and the
    cross-track distance ``d``. Mirrors ``fixedwing._guidance_course`` (which works P-centred)."""
    ux, uy = bx - ax_, by - ay
    leglen = math.hypot(ux, uy)
    ux, uy = ux / leglen, uy / leglen
    proj = (px - ax_) * ux + (py - ay) * uy  # (P - A)·u
    fx, fy = ax_ + proj * ux, ay + proj * uy
    d = math.hypot(px - fx, py - fy)
    ahead = math.sqrt(max(0.0, _L1_DISTANCE * _L1_DISTANCE - d * d))
    return (fx, fy), (fx + ahead * ux, fy + ahead * uy), d


def _capture_track(e0: float, n0: float, a_ll: tuple[float, float], b_ll: tuple[float, float],
                   tmax: float = 60.0, dt: float = 0.5) -> tuple[list[float], list[float]]:
    """A fixed-wing starting ``(e0, n0)`` off a leg ``A -> B``, tracking it with L1 (real
    ``FixedWing`` + ``MotionCommand`` leg setpoint, no wind): its ground track."""
    s = AircraftState(id="C", lat=_latlon(e0, n0)[0], lon=_latlon(e0, n0)[1],
                      trk=0.0, gs=CRUISE, yaw=0.0, bank=0.0)
    cmd = MotionCommand(target_position=b_ll, target_leg_start=a_ll, target_airspeed=CRUISE)
    xs, ys, t = [], [], 0.0
    while t < tmax:
        e, n = _enu(s.lat, s.lon)
        xs.append(e)
        ys.append(n)
        s = _FW.step(s, cmd, SMALL_FIXEDWING, dt)
        t += dt
    return xs, ys


def l1_figure(out: Path) -> None:
    fig, (a_geo, a_cap) = plt.subplots(1, 2, figsize=(11.5, 5.8))

    # --- left: the L1 construction for one off-track aircraft ---
    ax_, ay, bx, by = 0.0, 0.0, 0.0, 300.0  # leg A -> B, due north
    px, py = 50.0, 150.0  # aircraft, 50 m off the line (inside L1 = 80 m)
    (fx, fy), (rx, ry), d = _l1_reference(px, py, ax_, ay, bx, by)

    a_geo.plot([ax_, bx], [ay, by], color=GREY, ls="--", lw=1.4, zorder=1)
    a_geo.plot(ax_, ay, "*", color="0.2", ms=14, zorder=4)
    a_geo.plot(bx, by, "*", color="0.2", ms=14, zorder=4)
    a_geo.annotate("A", (ax_, ay), textcoords="offset points", xytext=(-15, -3), fontsize=11)
    a_geo.annotate("B", (bx, by), textcoords="offset points", xytext=(-15, -3), fontsize=11)
    a_geo.add_patch(plt.Circle((px, py), _L1_DISTANCE, fill=False, color=BLUE, lw=1.4, alpha=0.7))
    a_geo.plot([px, fx], [py, fy], color=RED, lw=1.6, zorder=3)  # cross-track d
    a_geo.annotate(f"d = {d:.0f} m", ((px + fx) / 2, (py + fy) / 2),
                   textcoords="offset points", xytext=(-8, -15), color=RED, fontsize=9)
    a_geo.annotate(f"L1 = {_L1_DISTANCE:.0f} m", (px, py - _L1_DISTANCE),
                   textcoords="offset points", xytext=(6, -12), color=BLUE, fontsize=9)
    a_geo.annotate("", xy=(rx, ry), xytext=(px, py),
                   arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=2.2))  # commanded course
    a_geo.annotate(r"$\chi_{cmd}$", ((px + rx) / 2, (py + ry) / 2),
                   textcoords="offset points", xytext=(7, -1), color=GREEN, fontsize=11)
    marks = (
        ((px, py), "aircraft", 8, -3), ((fx, fy), "foot F", -40, 4), ((rx, ry), "ref R", -38, 2),
    )
    for xy, lbl, dx, dy in marks:
        a_geo.plot(*xy, "o", color="0.2", ms=5, zorder=4)
        a_geo.annotate(lbl, xy, textcoords="offset points", xytext=(dx, dy), fontsize=9)
    a_geo.set_xlim(-120, 175)
    a_geo.set_ylim(-45, 320)
    a_geo.set_aspect("equal")
    a_geo.set_xlabel("east [m]")
    a_geo.set_ylabel("north [m]")
    a_geo.set_title("The L1 construction", fontsize=10)

    # --- right: cross-track capture of a due-north leg from either side ---
    a_ll, b_ll = _latlon(0.0, 0.0), _latlon(0.0, 600.0)
    a_cap.plot([0.0, 0.0], [0.0, 420.0], color=GREY, ls="--", lw=1.4, label="leg A→B", zorder=1)
    for e0, col in ((110.0, ORANGE), (-110.0, BLUE)):
        xs, ys = _capture_track(e0, 0.0, a_ll, b_ll, tmax=24.0)
        a_cap.plot(xs, ys, color=col, lw=2.2, zorder=3)
        a_cap.plot(xs[0], ys[0], "^", color=col, ms=9, zorder=4)
    a_cap.set_aspect("equal")
    a_cap.set_xlim(-175, 175)
    a_cap.set_ylim(-30, 420)
    a_cap.set_xlabel("east [m]")
    a_cap.set_ylabel("north [m]")
    a_cap.set_title("Cross-track capture, from either side", fontsize=10)
    a_cap.legend(fontsize=9, loc="lower right")

    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print("wrote", out)


def main() -> None:
    IMG.mkdir(parents=True, exist_ok=True)
    mission_figure(IMG / "autopilot-mission.png")
    l1_figure(IMG / "autopilot-l1.png")


if __name__ == "__main__":
    main()
