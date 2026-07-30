"""Phase-5c demo: a fixed-wing under wind — trochoidal ground tracks, and the contrast with a
multirotor.

A constant-airspeed fixed-wing turning at constant bank traces a **circle in the air frame** but a
**trochoid over the ground** (the paper's Fig. 4): each revolution is displaced by ``wind ×
period``. Ground speed varies with heading (Eq 4 — fastest downwind, slowest upwind) and the
airframe crabs to make good a ground course (Eq 3). The airframe difference the wind exposes: a
**multirotor can null its ground velocity** (hover into wind), a **fixed-wing cannot** — it is
always moving, so the best it can do to "hold" a point is a min-radius loiter.

Writes ``vault/observations/img/wind-multirotor-vs-fixedwing.png``.

    PYTHONPATH=. python scripts/wind_fixedwing_demo.py
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.kinematics import FixedWing, MotionCommand, Multirotor  # noqa: E402
from opencdarr.performance import M600, SMALL_FIXEDWING  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402
from opencdarr.wind import WindField  # noqa: E402
from scripts.windviz import draw_wind_arrow  # noqa: E402

LAT0, LON0, DT = 52.0, 4.0, 0.05
V = 17.0
WIND = WindField.from_met(270.0, 6.0)  # 6 m/s from the west (air moves east)
_FW, _MR = FixedWing(), Multirotor()


def _enu(lat: float, lon: float) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(LAT0, LON0, lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def turn_rows(tmax: float = 26.0) -> list[tuple[float, ...]]:
    """A continuous max-bank right turn: (t, ground e/n, air e/n, heading psi, V_GS, crab)."""
    s = AircraftState(id="F", lat=LAT0, lon=LON0, trk=0.0, gs=V, yaw=0.0, bank=0.0)
    we, wn = WIND.components()
    rows: list[tuple[float, ...]] = []
    t = 0.0
    while t < tmax:
        e, n = _enu(s.lat, s.lon)
        crab = ((s.yaw or 0.0) - s.trk + 180.0) % 360.0 - 180.0
        rows.append((t, e, n, e - we * t, n - wn * t, s.yaw or 0.0, s.gs, crab))
        cmd = MotionCommand(target_airspeed_direction=((s.yaw or 0.0) + 90.0) % 360.0,
                            target_airspeed=V)
        s = _FW.step(s, cmd, SMALL_FIXEDWING, DT, WIND)
        t += DT
    return rows


def loiter_rows(tmax: float = 60.0) -> list[tuple[float, ...]]:
    """A fixed-wing loitering about the origin (it cannot stop): (t, e, n, V_GS)."""
    start_lat, start_lon = geo.forward(LAT0, LON0, 90.0, 90.0)  # start ~on the orbit, 90 m east
    s = AircraftState(id="F", lat=start_lat, lon=start_lon, trk=0.0, gs=V, yaw=0.0, bank=0.0)
    cmd = MotionCommand(target_position=(LAT0, LON0), target_loiter_radius=90.0, target_airspeed=V)
    rows: list[tuple[float, ...]] = []
    t = 0.0
    while t < tmax:
        e, n = _enu(s.lat, s.lon)
        rows.append((t, e, n, s.gs))
        s = _FW.step(s, cmd, SMALL_FIXEDWING, DT, WIND)
        t += DT
    return rows


def hover_rows(tmax: float = 60.0) -> list[tuple[float, ...]]:
    """A multirotor commanded zero ground velocity — hovers into wind: (t, e, n, V_GS)."""
    s = AircraftState(id="M", lat=LAT0, lon=LON0, trk=0.0, gs=0.0)
    cmd = MotionCommand.from_track_speed(0.0, 0.0)
    rows: list[tuple[float, ...]] = []
    t = 0.0
    while t < tmax:
        e, n = _enu(s.lat, s.lon)
        rows.append((t, e, n, s.gs))
        s = _MR.step(s, cmd, M600, DT, WIND)
        t += DT
    return rows


def _col(rows: list[tuple[float, ...]], k: int) -> list[float]:
    return [r[k] for r in rows]


def plot(turn: list[tuple[float, ...]], loiter: list[tuple[float, ...]],
         hover: list[tuple[float, ...]], out: Path) -> None:
    fig, ax = plt.subplots(2, 2, figsize=(13.0, 10.5))

    # --- circle in air, trochoid over ground ---
    a = ax[0, 0]
    a.plot(_col(turn, 1), _col(turn, 2), color="tab:orange", lw=2.4,
           label="ground track (trochoid)")
    a.plot(_col(turn, 3), _col(turn, 4), color="tab:blue", lw=1.6, ls="--",
           label="air-frame path (circle, drift removed)")
    a.scatter([turn[0][1]], [turn[0][2]], color="k", marker="^", s=55, zorder=5)
    draw_wind_arrow(a, WIND, (30.0, -70.0), 25.0)
    a.set_aspect("equal", adjustable="datalim")
    a.set_xlabel("East [m]")
    a.set_ylabel("North [m]")
    a.set_title("Constant-bank turn: a circle in the air, a trochoid over the ground")
    a.grid(True, alpha=0.3)
    a.legend(fontsize=8, loc="best")

    # --- V_GS(psi) and crab(psi): sort by heading so the multi-revolution samples collapse onto
    #     the single Eq-4 / Eq-3 curve (avoids straight lines drawn where heading wraps 360->0) ---
    a = ax[0, 1]
    order = sorted(range(len(turn)), key=lambda i: turn[i][5])
    psi_s = [turn[i][5] for i in order]
    a.plot(psi_s, [turn[i][6] for i in order], color="tab:orange", lw=2.0, label="V_GS")
    a.axhline(V, color="0.5", ls=":", lw=1.0, label=f"airspeed V_TAS = {V:.0f}")
    a.set_xlabel("heading psi [deg]")
    a.set_ylabel("ground speed [m/s]")
    a2 = a.twinx()
    a2.plot(psi_s, [turn[i][7] for i in order], color="tab:green", lw=1.4, ls="--",
            label="crab psi−trk")
    a2.set_ylabel("crab [deg]")
    a.set_title("V_GS varies with heading (Eq 4): fastest downwind, slowest upwind")
    lines1, labels1 = a.get_legend_handles_labels()
    lines2, labels2 = a2.get_legend_handles_labels()
    a.legend(lines1 + lines2, labels1 + labels2, fontsize=8, loc="best")
    a.grid(True, alpha=0.3)

    # --- station-keeping contrast ---
    a = ax[1, 0]
    a.plot(_col(loiter, 1), _col(loiter, 2), color="tab:orange", lw=2.0,
           label="fixed-wing: min-radius loiter (can't stop)")
    a.plot(_col(hover, 1), _col(hover, 2), color="tab:blue", lw=2.0,
           label="multirotor: hovers into wind (holds point)")
    a.scatter([0], [0], color="k", marker="x", s=60, zorder=5, label="target point")
    draw_wind_arrow(a, WIND, (60.0, 60.0), 22.0)
    a.set_aspect("equal", adjustable="datalim")
    a.set_xlabel("East [m]")
    a.set_ylabel("North [m]")
    a.set_title("Holding a point in wind: the multirotor can, the fixed-wing cannot")
    a.grid(True, alpha=0.3)
    a.legend(fontsize=8, loc="best")

    # --- ground speed over time in the station-keeping contrast ---
    a = ax[1, 1]
    a.plot(_col(loiter, 0), _col(loiter, 3), color="tab:orange", lw=2.0, label="fixed-wing loiter")
    a.plot(_col(hover, 0), _col(hover, 3), color="tab:blue", lw=2.0, label="multirotor hover")
    a.axhline(0.0, color="0.5", ls=":", lw=1.0)
    a.set_xlabel("t [s]")
    a.set_ylabel("ground speed [m/s]")
    a.set_title("Ground speed: the multirotor can null it, the fixed-wing never does")
    a.grid(True, alpha=0.3)
    a.legend(fontsize=8, loc="best")

    fig.suptitle(
        f"Phase 5c: a fixed-wing under a {WIND.speed:.0f} m/s west wind — trochoidal tracks, "
        "and what only a multirotor can do",
        fontsize=12,
    )
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


def main() -> None:
    turn, loiter, hover = turn_rows(), loiter_rows(), hover_rows()
    gs = _col(turn, 6)
    print(f"turn: V_GS range [{min(gs):.1f}, {max(gs):.1f}] m/s about V_TAS {V:.0f} "
          f"(±V_WS {WIND.speed:.0f} expected)")
    print(f"multirotor hover: final ground speed {hover[-1][3]:.3f} m/s (station-kept)")
    print(f"fixed-wing loiter: final ground speed {loiter[-1][3]:.2f} m/s (always moving)")
    img = "vault/observations/img/wind-multirotor-vs-fixedwing.png"
    out = Path(__file__).resolve().parents[1] / img
    plot(turn, loiter, hover, out)


if __name__ == "__main__":
    main()
