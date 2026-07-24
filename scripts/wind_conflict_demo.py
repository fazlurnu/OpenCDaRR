"""Phase-5d demo: a fixed-wing conflict resolving, in still air vs in wind.

Two fixed-wings meet in a 90° crossing conflict and both run detect → resolve → recover
(StateBased + MVP + PastCPA), the MVP avoidance velocity projected onto each airframe's
course/airspeed channels (``project_to_fixedwing``). The same conflict is flown **twice** — once in
still air, once in a steady crosswind — to show that the DAA still clears (min-sep ≥ rpz), with the
wind crabbing and bending the resolution trajectories over the ground.

Writes ``vault/observations/img/wind-conflict-resolution.png``.

    PYTHONPATH=. python scripts/wind_conflict_demo.py
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.cd import StateBased  # noqa: E402
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.crr import PastCPA  # noqa: E402
from opencdarr.dynamics import FixedWing, MotionCommand  # noqa: E402
from opencdarr.performance import SMALL_FIXEDWING as P  # noqa: E402
from opencdarr.scenario import create_conflict  # noqa: E402
from opencdarr.separation import INACTIVE, SeparationManager, project_to_fixedwing  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402
from opencdarr.wind import NO_WIND, WindField  # noqa: E402
from scripts.windviz import draw_wind_arrow  # noqa: E402

LAT0, LON0, DT, BCAST, T_MAX = 52.0, 4.0, 0.2, 1.0, 70.0
RPZ, LOOKAHEAD = 50.0, 120.0
GS, DPSI, TLOS, MARGIN = 17.0, 90.0, 60.0, 1.25
# From the SW at 6 m/s: for this crossing the wind *tightens* the encounter (a fixed-wing holds its
# commanded course but its ground speed shifts with heading under wind, moving the closure). Most
# bearings instead widen the miss; this is the instructive "wind makes it harder" case.
WIND = WindField.from_met(225.0, 6.0)
_FW = FixedWing()


def _enu(lat: float, lon: float) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(LAT0, LON0, lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def run(wind: WindField) -> list[tuple[float, ...]]:
    """Both fixed-wings resolve the crossing in ``wind``; capture per-step trajectory rows."""
    own = AircraftState(id="OWN", lat=LAT0, lon=LON0, trk=0.0, gs=GS, yaw=0.0, bank=0.0)
    intr = create_conflict(own, intr_id="INT", dpsi=DPSI, dcpa=0.0, tlos=TLOS, rpz=RPZ, side=1)
    sep = SeparationManager()
    det, res, rec = StateBased(), MVP(margin=MARGIN), PastCPA(bouncing_guard=True)
    nom_own = MotionCommand.from_track_speed(own.trk, own.gs)
    nom_intr = MotionCommand.from_track_speed(intr.trk, intr.gs)

    def adapt(cmd: MotionCommand) -> MotionCommand:
        return project_to_fixedwing(cmd, P)

    mem_own = mem_intr = INACTIVE
    cmd_own, cmd_intr = adapt(nom_own), adapt(nom_intr)
    rows: list[tuple[float, ...]] = []
    t, nb = 0.0, 0.0
    while t < T_MAX:
        if t + 1e-9 >= nb:
            cmd_own, mem_own = sep.step(
                own, [intr], nom_own, mem_own, RPZ, LOOKAHEAD, det, res, rec, adapt)
            cmd_intr, mem_intr = sep.step(
                intr, [own], nom_intr, mem_intr, RPZ, LOOKAHEAD, det, res, rec, adapt)
            nb += BCAST
        oe, on = _enu(own.lat, own.lon)
        ie, in_ = _enu(intr.lat, intr.lon)
        _, sepd = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
        rows.append((t, oe, on, ie, in_, sepd,
                     float(mem_own.resolving or mem_intr.resolving)))
        own = _FW.step(own, cmd_own, P, DT, wind)
        intr = _FW.step(intr, cmd_intr, P, DT, wind)
        t += DT
    return rows


def _col(rows: list[tuple[float, ...]], k: int) -> list[float]:
    return [r[k] for r in rows]


def _tracks(ax: plt.Axes, rows: list[tuple[float, ...]], wind: WindField, title: str) -> None:
    sep = _col(rows, 5)
    i_cpa = min(range(len(sep)), key=lambda i: sep[i])
    ax.plot(_col(rows, 1), _col(rows, 2), color="tab:orange", lw=2.2, label="OWN (fixed-wing)")
    ax.plot(_col(rows, 3), _col(rows, 4), color="tab:blue", lw=2.2, label="INT (fixed-wing)")
    ax.scatter([rows[0][1]], [rows[0][2]], color="tab:orange", marker="^", s=55, zorder=5)
    ax.scatter([rows[0][3]], [rows[0][4]], color="tab:blue", marker="^", s=55, zorder=5)
    mid_e = (rows[i_cpa][1] + rows[i_cpa][3]) / 2
    mid_n = (rows[i_cpa][2] + rows[i_cpa][4]) / 2
    ax.add_patch(plt.Circle((rows[i_cpa][3], rows[i_cpa][4]), RPZ, fill=False, ls="--",
                            color="tab:red", lw=1.0, alpha=0.6))
    ax.annotate(f"min sep {sep[i_cpa]:.0f} m\n(rpz {RPZ:.0f})", (mid_e, mid_n),
                textcoords="offset points", xytext=(8, 8), fontsize=8, color="tab:red")
    if wind.speed > 0:  # top-right corner, clear of the tracks and the CPA annotation
        max_e = max(_col(rows, 1) + _col(rows, 3))
        max_n = max(_col(rows, 2) + _col(rows, 4))
        draw_wind_arrow(ax, wind, (max_e - 120.0, max_n), 70.0)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="best")


def plot(calm: list[tuple[float, ...]], windy: list[tuple[float, ...]], out: Path) -> None:
    fig = plt.figure(figsize=(14.0, 6.0))
    a0 = fig.add_subplot(1, 3, 1)
    a1 = fig.add_subplot(1, 3, 2)
    a2 = fig.add_subplot(1, 3, 3)
    _tracks(a0, calm, NO_WIND, "Still air: the conflict resolves")
    _tracks(a1, windy, WIND,
            f"{WIND.speed:.0f} m/s wind from {WIND.coming_from:.0f}°: still resolves, crabbed")

    a2.plot(_col(calm, 0), _col(calm, 5), color="0.4", lw=2.0, label="still air")
    a2.plot(_col(windy, 0), _col(windy, 5), color="tab:green", lw=2.0, label="with wind")
    a2.axhline(RPZ, color="tab:red", ls="--", lw=1.0, label=f"rpz = {RPZ:.0f} m")
    a2.set_xlabel("t [s]")
    a2.set_ylabel("separation [m]")
    a2.set_title("Separation over time — both clear the rpz")
    a2.grid(True, alpha=0.3)
    a2.legend(fontsize=8)

    fig.suptitle(
        "Phase 5d: a fixed-wing crossing conflict resolves in still air and in a crosswind "
        "(MVP + Past-CPA, avoidance velocity projected to course/airspeed)",
        fontsize=12,
    )
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


def main() -> None:
    calm, windy = run(NO_WIND), run(WIND)
    print(f"still air: min sep {min(_col(calm, 5)):.1f} m (rpz {RPZ:.0f})")
    print(f"with wind: min sep {min(_col(windy, 5)):.1f} m (rpz {RPZ:.0f})")
    img = "vault/observations/img/wind-conflict-resolution.png"
    out = Path(__file__).resolve().parents[1] / img
    plot(calm, windy, out)


if __name__ == "__main__":
    main()
