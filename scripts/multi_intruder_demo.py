"""Phase-6a demo: MVP sums, VO takes the union — the multi-intruder resolution, in velocity space.

An ownship (heading north) is in simultaneous conflict with two intruders crossing from ±60°. Each
intruder forbids a **cone** of ownship velocities (its velocity obstacle). The panels show why the
two resolvers compose differently over a *set* of conflicts (ADR 0004 / Phase 6):

- **left (velocity space, the SSD):** the two cones and the resolved velocities. The **VO** result
  lands **outside both cones** (on the union boundary) — the shortest way out of the union. The
  **MVP** velocity is the **sum** of the two pairwise avoidance vectors; superposition partly
  cancels, so it stays **inside** the union — it under-clears a symmetric double conflict.
- **right (ground frame):** propagating the resolved velocities, VO opens **both** misses to the
  resolution zone while MVP's summed correction falls short.

Writes ``vault/observations/img/multi-intruder-vo-vs-mvp.png``.

    PYTHONPATH=. python scripts/multi_intruder_demo.py
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.cr import MVP, VO  # noqa: E402
from opencdarr.cr.vo import _Cone, _cone  # noqa: E402
from opencdarr.relative import velocity_enu  # noqa: E402
from opencdarr.scenario import create_conflict  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

RPZ, MARGIN = 50.0, 1.05
RPZ_EFF = RPZ * MARGIN


def _setup() -> tuple[AircraftState, AircraftState, AircraftState]:
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0)
    i1 = create_conflict(own, intr_id="I1", dpsi=60.0, dcpa=0.0, tlos=60.0, rpz=RPZ, side=1)
    i2 = create_conflict(own, intr_id="I2", dpsi=300.0, dcpa=0.0, tlos=60.0, rpz=RPZ, side=-1)
    return own, i1, i2


def _miss(own: AircraftState, intr: AircraftState, ve: float, vn: float) -> float:
    qdr, dist = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
    q = math.radians(qdr)
    rx, ry = dist * math.sin(q), dist * math.cos(q)
    vx = intr.gs * math.sin(math.radians(intr.trk)) - ve
    vy = intr.gs * math.cos(math.radians(intr.trk)) - vn
    t_cpa = -(rx * vx + ry * vy) / (vx * vx + vy * vy)
    return math.hypot(rx + vx * t_cpa, ry + vy * t_cpa)


def _cone_polygon(cone: _Cone, reach: float) -> tuple[list[float], list[float]]:
    """The filled cone as a triangle: apex + the two edge directions extended by ``reach``."""
    xs, ys = [cone.apex_e], [cone.apex_n]
    for sign in (-1.0, 1.0):
        ang = cone.bearing + sign * cone.half
        xs.append(cone.apex_e + reach * math.sin(ang))
        ys.append(cone.apex_n + reach * math.cos(ang))
    return xs, ys


def plot(out: Path) -> None:
    own, i1, i2 = _setup()
    vox, voy = velocity_enu(own)
    c1, c2 = _cone(own, i1, RPZ_EFF), _cone(own, i2, RPZ_EFF)
    assert c1 is not None and c2 is not None
    mvp = MVP(margin=MARGIN).resolve(own, [i1, i2], RPZ).target_velocity
    vo = VO(margin=MARGIN).resolve(own, [i1, i2], RPZ).target_velocity

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13.0, 6.0))

    # --- velocity space (the solution-space diagram) ---
    for c, col, lab in ((c1, "tab:blue", "VO cone, intruder 1"), (c2, "tab:green", "VO cone, i2")):
        xs, ys = _cone_polygon(c, 30.0)
        a1.fill(xs, ys, color=col, alpha=0.20, label=lab)
        a1.plot(xs[1:], ys[1:], color=col, lw=0.8, alpha=0.5)
    a1.scatter([vox], [voy], color="k", marker="o", s=45, zorder=5, label="current velocity")
    mvp_in = "inside union" if c1.contains(*mvp) or c2.contains(*mvp) else "outside"
    vo_in = "inside union" if c1.contains(*vo) or c2.contains(*vo) else "outside union"
    a1.scatter([mvp[0]], [mvp[1]], color="tab:red", marker="s", s=70, zorder=6,
               label=f"MVP (sum) — {mvp_in}")
    a1.scatter([vo[0]], [vo[1]], color="tab:orange", marker="*", s=180, zorder=6,
               label=f"VO (union) — {vo_in}")
    a1.set_aspect("equal", adjustable="box")
    a1.set_xlim(-14, 14)
    a1.set_ylim(-4, 16)
    a1.set_xlabel("East velocity [m/s]")
    a1.set_ylabel("North velocity [m/s]")
    a1.set_title("Velocity space: MVP sums into the union, VO leaves it")
    a1.grid(True, alpha=0.3)
    a1.legend(fontsize=7.5, loc="lower center")

    # --- ground frame: the miss distances the two resolutions achieve ---
    labels = ["intruder 1", "intruder 2"]
    mvp_miss = [_miss(own, i1, *mvp), _miss(own, i2, *mvp)]
    vo_miss = [_miss(own, i1, *vo), _miss(own, i2, *vo)]
    x = range(len(labels))
    a2.bar([i - 0.2 for i in x], mvp_miss, 0.4, color="tab:red", label="MVP (sum)")
    a2.bar([i + 0.2 for i in x], vo_miss, 0.4, color="tab:orange", label="VO (union)")
    a2.axhline(RPZ, color="tab:red", ls="--", lw=1.2, label=f"rpz = {RPZ:.0f} m")
    a2.axhline(RPZ_EFF, color="0.5", ls=":", lw=1.0, label=f"margin·rpz = {RPZ_EFF:.0f} m")
    for i, (m, v) in enumerate(zip(mvp_miss, vo_miss, strict=True)):
        a2.annotate(f"{m:.0f}", (i - 0.2, m + 1), ha="center", fontsize=8, color="tab:red")
        a2.annotate(f"{v:.0f}", (i + 0.2, v + 1), ha="center", fontsize=8, color="tab:orange")
    a2.set_xticks(list(x))
    a2.set_xticklabels(labels)
    a2.set_ylabel("resolved miss distance [m]")
    a2.set_title("Both misses: VO clears the zone, the MVP sum under-clears")
    a2.grid(True, alpha=0.3, axis="y")
    a2.legend(fontsize=8)

    fig.suptitle(
        "Phase 6a: multi-intruder resolution — MVP sums pairwise avoidance vectors, "
        "VO takes the shortest way out of the union of cones",
        fontsize=12,
    )
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")
    print(f"MVP miss: {mvp_miss[0]:.1f}, {mvp_miss[1]:.1f} m  (rpz {RPZ:.0f})")
    print(f"VO  miss: {vo_miss[0]:.1f}, {vo_miss[1]:.1f} m")


def main() -> None:
    img = "vault/observations/img/multi-intruder-vo-vs-mvp.png"
    out = Path(__file__).resolve().parents[1] / img
    plot(out)


if __name__ == "__main__":
    main()
