"""Phase-4d demo: a multirotor flies a 3-waypoint mission (WaypointAutopilot + Multirotor).

Shows the navigator/controller split (ADR 0014): the autopilot only emits a position setpoint for
the active waypoint; the airframe's position tracker flies *through* intermediate waypoints
(capture radius) and decelerates to a hover at the final one via the stopping-distance law. Two
panels: the ground track coloured by active leg, and the ground-speed profile.

Writes ``vault/observations/img/mission-waypoints.png``.

    python scripts/mission_demo.py
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.autopilot import GuidanceMemory, WaypointAutopilot  # noqa: E402
from opencdarr.dynamics import Multirotor  # noqa: E402
from opencdarr.mission import Mission, Waypoint  # noqa: E402
from opencdarr.performance import M600  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

LAT0, LON0, DT, CAP = 52.0, 4.0, 0.1, 30.0
_MR = Multirotor()
_LEG_COLORS = ("tab:blue", "tab:orange", "tab:green")


def _enu(lat: float, lon: float) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(LAT0, LON0, lat, lon)
    return dist * math.sin(math.radians(qdr)), dist * math.cos(math.radians(qdr))


def run() -> tuple[list[tuple[float, float, float, float, int]], list[tuple[float, float]]]:
    """Fly the plan; return per-step (t, east, north, gs, leg) rows and the waypoints in ENU."""
    a = geo.forward(LAT0, LON0, 0.0, 300.0)  # 300 m north
    b = geo.forward(a[0], a[1], 90.0, 300.0)  # then 300 m east
    c = geo.forward(b[0], b[1], 180.0, 300.0)  # then 300 m south
    mission = Mission(flight_plan=(Waypoint(*a), Waypoint(*b), Waypoint(*c)))

    ap = WaypointAutopilot(mission, capture_radius=CAP)
    gm = GuidanceMemory()
    s = AircraftState(id="M", lat=LAT0, lon=LON0, trk=0.0, gs=0.0)
    rows = []
    for i in range(1400):
        cmd, gm = ap.step(s, gm, M600)
        s = _MR.step(s, cmd, M600, DT)
        e, n = _enu(s.lat, s.lon)
        rows.append((i * DT, e, n, s.gs, gm.leg_index))
        _, dfin = geo.qdrdist(s.lat, s.lon, c[0], c[1])
        if dfin < 0.5 and s.gs < 0.05 and i > 50:
            break
    return rows, [_enu(*a), _enu(*b), _enu(*c)]


def plot(rows, wps_en, out: Path) -> None:
    t = [r[0] for r in rows]
    e, n, gs, leg = ([r[k] for r in rows] for k in (1, 2, 3, 4))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 5.8))

    for i_leg in (0, 1, 2):
        xs = [e[i] for i in range(len(e)) if leg[i] == i_leg]
        ys = [n[i] for i in range(len(n)) if leg[i] == i_leg]
        ax1.plot(xs, ys, color=_LEG_COLORS[i_leg], lw=2.2, label=f"leg {i_leg + 1}", zorder=3)
    for k, (we, wn) in enumerate(wps_en):
        ax1.scatter([we], [wn], color="k", marker="s", s=45, zorder=5)
        ax1.add_patch(plt.Circle((we, wn), CAP, fill=False, ls="--", color="0.6", lw=1.0))
        ax1.annotate(f"WP{k + 1}", (we, wn), textcoords="offset points", xytext=(8, 8), fontsize=9)
    ax1.scatter([0], [0], color="k", marker="^", s=70, zorder=5)
    ax1.plot([], [], ls="--", color="0.6", label=f"capture radius ({CAP:.0f} m)")
    ax1.set_title("Multirotor flies a 3-waypoint mission (ADR 0014)", fontsize=11)
    ax1.set_xlabel("East [m]")
    ax1.set_ylabel("North [m]")
    ax1.set_aspect("equal")
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=8, loc="upper right")

    ax2.plot(t, gs, color="tab:purple", lw=2.0)
    ax2.axhline(M600.v_max, ls=":", color="0.5", lw=1.0, label=f"v_max ({M600.v_max:.0f} m/s)")
    for i in range(1, len(leg)):
        if leg[i] != leg[i - 1]:
            ax2.axvline(t[i], ls="--", color="0.7", lw=1.0)
    ax2.set_title("Ground speed: cruise, fly-through captures, decel to a hover", fontsize=11)
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("ground speed [m/s]")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8, loc="lower left")

    fig.suptitle("Phase 4d: multirotor waypoint mission", fontsize=13)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=120)
    print(f"wrote {out}")


def main() -> None:
    rows, wps_en = run()
    out = Path(__file__).resolve().parents[1] / "vault/observations/img/mission-waypoints.png"
    plot(rows, wps_en, out)
    print(f"reached final waypoint and hovered at t={rows[-1][0]:.1f} s; "
          f"final gs={rows[-1][3]:.3f} m/s")


if __name__ == "__main__":
    main()
