"""Phase-4d demo: fixed-wing L1 leg-tracking with a mid-leg MVP resolution (far waypoint).

A fixed-wing tracks a leg to a waypoint 3 km ahead, is interrupted by a crossing intruder, resolves
with MVP, then the L1 tracker re-intercepts the *planned leg line* (not just the distant endpoint).
Cross-track error is pushed off during the avoidance and driven back to zero by L1 -- the point of
L1 over pure-pursuit.

NOTE: a fixed-wing cannot fly a raw velocity, so the MVP avoidance velocity is projected to a
(course, airspeed) setpoint by ``separation.project_to_fixedwing`` -- the production Phase-4e
velocity->course adapter (this demo's original ``_project_velocity`` prototype, now shipped;
ADR 0015). The mission / L1 half is all real Phase-4d code.

Writes ``vault/observations/img/fixedwing-l1-reintercept.png``.

    python scripts/l1_reintercept_demo.py
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
from opencdarr.kinematics import FixedWing, MotionCommand  # noqa: E402
from opencdarr.mission import Mission  # noqa: E402
from opencdarr.performance import SMALL_FIXEDWING as P  # noqa: E402
from opencdarr.scenario import create_conflict  # noqa: E402
from opencdarr.separation import INACTIVE, SeparationManager, project_to_fixedwing  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

LAT0, LON0, DT, BCAST = 52.0, 4.0, 0.1, 1.0
RPZ, LOOKAHEAD = 50.0, 18.0
_FW = FixedWing()


def _enu(lat: float, lon: float) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(LAT0, LON0, lat, lon)
    return dist * math.sin(math.radians(qdr)), dist * math.cos(math.radians(qdr))


def _project_velocity(cmd: MotionCommand) -> MotionCommand:
    """The Phase-4e adapter (ADR 0015): a resolver velocity -> a fixed-wing (course, airspeed)."""
    return project_to_fixedwing(cmd, P)


def run():
    """Run the encounter; return per-step rows and the far waypoint in ENU."""
    own = AircraftState(id="OWN", lat=LAT0, lon=LON0, trk=0.0, gs=17.0, yaw=0.0, bank=0.0)
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=0.0, tlos=30.0, rpz=RPZ, side=1)
    far_wp = geo.forward(LAT0, LON0, 0.0, 3000.0)  # 3 km straight ahead (north)

    ap = WaypointAutopilot(Mission(goto=(far_wp[0], far_wp[1])), cruise_airspeed=17.0)
    gm = GuidanceMemory()
    sep = SeparationManager()
    mem = INACTIVE
    det, res, rec = StateBased(), MVP(margin=1.2), PastCPA(bouncing_guard=True)
    intr_cmd = MotionCommand(target_course=intr.trk, target_airspeed=intr.gs)  # INT cruises
    cmd = MotionCommand(target_position=(far_wp[0], far_wp[1]), target_leg_start=(LAT0, LON0))

    rows = []
    t, nb = 0.0, 0.0
    while t < 120.0:
        if t + 1e-9 >= nb:
            nom, gm = ap.step(own, gm, P)
            nom = MotionCommand(target_position=nom.target_position,
                                target_leg_start=(LAT0, LON0), target_airspeed=17.0)
            raw, mem = sep.step(own, [intr], nom, mem, RPZ, LOOKAHEAD, det, res, rec)
            cmd = _project_velocity(raw)
            nb += BCAST
        oe, on = _enu(own.lat, own.lon)
        ie, in_ = _enu(intr.lat, intr.lon)
        _, sepd = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
        rows.append((t, oe, on, ie, in_, oe, sepd, mem.resolving))  # oe = cross-track (x=0 leg)
        own = _FW.step(own, cmd, P, DT)
        intr = _FW.step(intr, intr_cmd, P, DT)
        if on > 1500.0:
            break
        t += DT
    return rows, _enu(far_wp[0], far_wp[1])


def _seg(ax, xs, ys, mask, color, label):
    drawn = False
    for i in range(1, len(xs)):
        if mask[i]:
            ax.plot(xs[i - 1:i + 1], ys[i - 1:i + 1], color=color, lw=2.6,
                    label=label if not drawn else None, zorder=4)
            drawn = True


def plot(rows, far_en, out: Path) -> None:
    t = [r[0] for r in rows]
    oe, on, ie, in_, xt, sep, res = ([r[k] for r in rows] for k in (1, 2, 3, 4, 5, 6, 7))
    i_cpa = min(range(len(sep)), key=lambda i: sep[i])
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.8, 6.2),
                                   gridspec_kw={"width_ratios": [1, 1.15]})

    ax1.plot([0, far_en[0]], [0, far_en[1]], ls="--", color="0.6", lw=1.2,
             label="planned leg line", zorder=2)
    _seg(ax1, oe, on, [not r for r in res], "tab:blue", "mission (L1 leg-track)")
    _seg(ax1, oe, on, res, "tab:red", "MVP avoidance (offboard)")
    ax1.plot(ie, in_, color="0.55", lw=1.8, label="INT (crossing)", zorder=3)
    ax1.add_patch(plt.Circle((ie[i_cpa], in_[i_cpa]), RPZ, fill=False, ls="--", color="tab:red",
                             lw=1.0, alpha=0.6))
    ax1.annotate(f"min sep {sep[i_cpa]:.0f} m", (ie[i_cpa], in_[i_cpa]),
                 textcoords="offset points", xytext=(10, -22), fontsize=8, color="tab:red")
    ax1.annotate("L1 re-intercepts\nthe leg line", (5, 900), fontsize=9, color="tab:blue")
    ax1.annotate("goto WP 3 km\nfurther north", (2, 1080), fontsize=8)
    ax1.set_xlim(-70, 120)
    ax1.set_ylim(280, 1180)
    ax1.set_title("Fixed-wing: leg to a far waypoint, resolve with MVP,\n"
                  "then L1 re-intercepts (east axis exaggerated)", fontsize=10)
    ax1.set_xlabel("East [m]")
    ax1.set_ylabel("North [m]")
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=8, loc="lower right")

    ax2.plot(t, xt, color="tab:green", lw=2.0)
    ax2.axhline(0.0, ls=":", color="0.5", lw=1.0)
    in_res, s_t, banded = False, 0.0, False
    for i in range(len(t)):
        if res[i] and not in_res:
            in_res, s_t = True, t[i]
        if in_res and (i == len(t) - 1 or not res[i]):
            ax2.axvspan(s_t, t[i], color="tab:red", alpha=0.12,
                        label=None if banded else "MVP avoidance")
            in_res, banded = False, True
    ax2.annotate("pushed off track\nby avoidance", (t[i_cpa], max(xt) * 0.8), fontsize=8,
                 ha="center")
    ax2.annotate("L1 nulls\ncross-track", (t[-1] * 0.8, 5), fontsize=8, ha="center",
                 color="tab:green")
    ax2.set_title("Cross-track error from the planned leg\n(L1 re-intercept after the excursion)",
                  fontsize=11)
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("cross-track [m]  (east of the leg)")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8, loc="upper right")

    fig.suptitle("Phase 4d: fixed-wing L1 leg-tracking with a mid-leg MVP resolution", fontsize=13)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=120)
    print(f"wrote {out}")


def main() -> None:
    rows, far_en = run()
    img = "vault/observations/img/fixedwing-l1-reintercept.png"
    out = Path(__file__).resolve().parents[1] / img
    plot(rows, far_en, out)
    sep = [r[6] for r in rows]
    xt = [r[5] for r in rows]
    print(f"min sep = {min(sep):.1f} m (rpz {RPZ:.0f}); max |cross-track| = "
          f"{max(abs(x) for x in xt):.1f} m; final cross-track = {xt[-1]:.2f} m")


if __name__ == "__main__":
    main()
