"""Phase-4d demo: mission-flying multirotor -> crossing intruder -> MVP resolution -> resume.

OWN flies a goto mission north; INT crosses on a collision course. The SeparationManager overlays
MVP (the offboard interrupt) while the conflict is live and releases to the mission on recovery --
OWN then continues to its waypoint. The resume is *automatic*: the autopilot re-plans toward the
waypoint every tick, so releasing the override simply resumes the mission (no mode machine).

All real: WaypointAutopilot + SeparationManager(MVP, PastCPA) + Multirotor, no measurement noise
(a clean picture). Writes ``vault/observations/img/mission-resume.png``.

    python scripts/resume_demo.py
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.autopilot import GuidanceMemory, WaypointAutopilot  # noqa: E402
from opencdarr.cd import StateBased  # noqa: E402
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.crr import PastCPA  # noqa: E402
from opencdarr.dynamics import MotionCommand, Multirotor  # noqa: E402
from opencdarr.mission import Mission  # noqa: E402
from opencdarr.performance import M600  # noqa: E402
from opencdarr.scenario import create_conflict  # noqa: E402
from opencdarr.separation import INACTIVE, SeparationManager  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

LAT0, LON0, DT, BCAST = 52.0, 4.0, 0.1, 1.0
RPZ, LOOKAHEAD = 50.0, 12.0
_MR = Multirotor()


def _enu(lat: float, lon: float) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(LAT0, LON0, lat, lon)
    return dist * math.sin(math.radians(qdr)), dist * math.cos(math.radians(qdr))


def run():
    """Run the encounter; return per-step rows and the goto point in ENU."""
    own = AircraftState(id="OWN", lat=LAT0, lon=LON0, trk=0.0, gs=18.0)  # cruising north
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=0.0, tlos=20.0, rpz=RPZ, side=1)
    goto = geo.forward(LAT0, LON0, 0.0, 700.0)  # waypoint 700 m north, past the conflict

    ap = WaypointAutopilot(Mission(goto=(goto[0], goto[1])))
    gm = GuidanceMemory()
    sep = SeparationManager()
    mem = INACTIVE
    det, res, rec = StateBased(), MVP(margin=1.1), PastCPA(bouncing_guard=True)
    intr_cmd = MotionCommand.from_track_speed(intr.trk, intr.gs)  # INT cruises straight
    cmd_own = MotionCommand(target_position=(goto[0], goto[1]))

    rows = []
    t, nb = 0.0, 0.0
    while t < 60.0:
        if t + 1e-9 >= nb:
            nom_own, gm = ap.step(own, gm, M600)
            cmd_own, mem = sep.step(own, [intr], nom_own, mem, RPZ, LOOKAHEAD, det, res, rec)
            nb += BCAST
        oe, on = _enu(own.lat, own.lon)
        ie, in_ = _enu(intr.lat, intr.lon)
        _, sepdist = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
        rows.append((t, oe, on, ie, in_, sepdist, mem.resolving))
        own = _MR.step(own, cmd_own, M600, DT)
        intr = _MR.step(intr, intr_cmd, M600, DT)
        _, dgoto = geo.qdrdist(own.lat, own.lon, goto[0], goto[1])
        if dgoto < 1.0:
            break
        t += DT
    return rows, _enu(goto[0], goto[1])


def plot(rows, goto_en, out: Path) -> None:
    t = [r[0] for r in rows]
    oe, on, ie, in_, sep, resolving = ([r[k] for r in rows] for k in (1, 2, 3, 4, 5, 6))
    i_cpa = min(range(len(sep)), key=lambda i: sep[i])
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 6.0))

    def seg(xs, ys, mask, color, label):
        drawn = False
        for i in range(1, len(xs)):
            if mask[i]:
                ax1.plot(xs[i - 1:i + 1], ys[i - 1:i + 1], color=color, lw=2.6,
                         label=label if not drawn else None, zorder=4)
                drawn = True

    seg(oe, on, [not r for r in resolving], "tab:blue", "OWN -- mission (nominal)")
    seg(oe, on, resolving, "tab:red", "OWN -- MVP avoidance (offboard)")
    ax1.plot(ie, in_, color="0.55", lw=2.0, label="INT -- crossing traffic", zorder=3)
    ax1.scatter([oe[0]], [on[0]], color="tab:blue", marker="^", s=70, zorder=6)
    ax1.scatter([ie[0]], [in_[0]], color="0.4", marker="^", s=70, zorder=6)
    ax1.scatter([goto_en[0]], [goto_en[1]], color="k", marker="s", s=55, zorder=6)
    ax1.annotate("goto WP", goto_en, textcoords="offset points", xytext=(8, 6), fontsize=9)
    ax1.add_patch(plt.Circle((ie[i_cpa], in_[i_cpa]), RPZ, fill=False, ls="--", color="tab:red",
                             lw=1.0, alpha=0.6))
    ax1.annotate(f"min sep {sep[i_cpa]:.0f} m\n(rpz {RPZ:.0f} m)", (ie[i_cpa], in_[i_cpa]),
                 textcoords="offset points", xytext=(10, -28), fontsize=8, color="tab:red")
    ax1.set_title("OWN flies its mission, avoids the intruder with MVP, then resumes", fontsize=11)
    ax1.set_xlabel("East [m]")
    ax1.set_ylabel("North [m]")
    ax1.set_aspect("equal")
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=8, loc="lower right")

    ax2.plot(t, sep, color="tab:purple", lw=2.0)
    ax2.axhline(RPZ, ls="--", color="tab:red", lw=1.2, label=f"rpz ({RPZ:.0f} m)")
    in_res, start_t = False, 0.0
    for i in range(len(t)):
        if resolving[i] and not in_res:
            in_res, start_t = True, t[i]
        if in_res and (i == len(t) - 1 or not resolving[i]):
            ax2.axvspan(start_t, t[i], color="tab:red", alpha=0.12)
            in_res = False
    ax2.plot([], [], color="tab:red", alpha=0.3, lw=8, label="MVP avoidance active")
    ax2.set_title("Separation over time -- stays above rpz;\nred band = mission interrupted",
                  fontsize=11)
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("separation [m]")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8, loc="upper right")

    fig.suptitle("Phase 4d: mission -> intruder -> MVP resolution -> resume (multirotor)",
                 fontsize=13)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=120)
    print(f"wrote {out}")


def main() -> None:
    rows, goto_en = run()
    out = Path(__file__).resolve().parents[1] / "vault/observations/img/mission-resume.png"
    plot(rows, goto_en, out)
    sep = [r[5] for r in rows]
    avoid = sum(r[6] for r in rows)
    print(f"min sep = {min(sep):.1f} m (rpz {RPZ:.0f}); avoidance ticks = {avoid} / {len(rows)}")


if __name__ == "__main__":
    main()
