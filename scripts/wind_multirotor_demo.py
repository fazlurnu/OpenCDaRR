"""Phase-5b demo: a multirotor flying one ground-velocity command in two winds — one below its
envelope, one above.

``target_velocity`` is a **ground** velocity (5 m/s north here), but the ``v_max``/``ax`` envelope
is on **airspeed** (ADR 0016 / Phase-5 decision 4). A small multirotor (``v_max = 8 m/s``) is
commanded a constant 5 m/s north in a **crosswind from the west**, at two magnitudes straddling the
feasibility crossover ``√(5² + w²) ≤ v_max`` (``w ≲ 6.24``):

- **below envelope** (``w = 4 m/s``, required airspeed ≈ 6.4 ≤ 8): the multirotor **crabs** into
  the wind and its **ground track stays straight north** — the command is met exactly;
- **above envelope** (``w = 7 m/s``, required airspeed ≈ 8.6 > 8): the airspeed **clamps at
  ``v_max``**, the drift can't be fully cancelled, and the ground track **bows downwind (east)**.

Writes ``vault/observations/img/wind-multirotor-envelope.png``.

    PYTHONPATH=. python scripts/wind_multirotor_demo.py
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.kinematics import MotionCommand, Multirotor  # noqa: E402
from opencdarr.performance import Performance  # noqa: E402
from opencdarr.relative import ground_to_air, velocity_enu  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402
from opencdarr.wind import WindField  # noqa: E402
from scripts.windviz import draw_wind_arrow  # noqa: E402

LAT0, LON0, DT, T_MAX = 52.0, 4.0, 0.1, 60.0
GS_CMD = 5.0  # commanded ground speed, due north
PERF = Performance(v_max=8.0, v_min=-8.0, ax=4.0, yaw_rate_max=90.0)  # a small multirotor
WIND_BELOW = WindField.from_met(270.0, 4.0)  # crosswind from the west, within the envelope
WIND_ABOVE = WindField.from_met(270.0, 7.0)  # ... and beyond it
_MR = Multirotor()


def _enu(lat: float, lon: float) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(LAT0, LON0, lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def run(wind: WindField) -> list[tuple[float, ...]]:
    """Fly the constant 5 m/s-north ground command in ``wind``; capture per-step rows."""
    state = AircraftState(id="M", lat=LAT0, lon=LON0, trk=0.0, gs=GS_CMD)
    cmd = MotionCommand.from_track_speed(0.0, GS_CMD)
    rows: list[tuple[float, ...]] = []
    t = 0.0
    while t < T_MAX:
        e, n = _enu(state.lat, state.lon)
        ve, vn = velocity_enu(state)
        ae, an = ground_to_air((ve, vn), wind)  # the airspeed vector the airframe is flying
        rows.append((t, e, n, ve, vn, ae, an))
        state = _MR.step(state, cmd, PERF, DT, wind)
        t += DT
    return rows


def _col(rows: list[tuple[float, ...]], k: int) -> list[float]:
    return [r[k] for r in rows]


def _triangle(ax: plt.Axes, rows: list[tuple[float, ...]], wind: WindField, color: str) -> None:
    """The steady-state wind triangle: airspeed vector + wind = ground velocity."""
    ve, vn, ae, an = rows[-1][3], rows[-1][4], rows[-1][5], rows[-1][6]
    we, wn = wind.components()
    ax.annotate("", xy=(ae, an), xytext=(0, 0),
                arrowprops={"arrowstyle": "-|>", "color": color, "lw": 2.2})
    ax.annotate("", xy=(ae + we, an + wn), xytext=(ae, an),
                arrowprops={"arrowstyle": "-|>", "color": "0.5", "lw": 2.0})
    ax.annotate("", xy=(ve, vn), xytext=(0, 0),
                arrowprops={"arrowstyle": "-|>", "color": "k", "lw": 1.4, "ls": ":"})
    ax.text(ae, an, f" air {math.hypot(ae, an):.1f}", color=color, fontsize=8, ha="right")


def plot(below: list[tuple[float, ...]], above: list[tuple[float, ...]], out: Path) -> None:
    fig, ax = plt.subplots(2, 2, figsize=(13.0, 9.5))

    # --- ground tracks + the wind arrow ---
    a = ax[0, 0]
    a.plot(_col(below, 1), _col(below, 2), color="tab:blue", lw=2.4,
           label="below envelope (w=4): crabs, straight")
    a.plot(_col(above, 1), _col(above, 2), color="tab:red", lw=2.4,
           label="above envelope (w=7): clamps, drifts")
    a.scatter([0], [0], color="k", marker="^", s=60, zorder=5)
    a.axvline(0.0, color="0.85", lw=0.8, zorder=0)
    draw_wind_arrow(a, WIND_ABOVE, (18.0, 250.0), 22.0)
    a.set_xlim(-15, 55)
    a.set_xlabel("East [m]")
    a.set_ylabel("North [m]")
    a.set_title("Ground tracks under a west crosswind (command: 5 m/s north)")
    a.grid(True, alpha=0.3)
    a.legend(fontsize=8, loc="lower right")

    # --- the wind triangle (airspeed + wind = ground), steady state ---
    a = ax[0, 1]
    circ = plt.Circle((0, 0), PERF.v_max, fill=False, ls="--", color="0.6", lw=1.0)
    a.add_patch(circ)
    _triangle(a, below, WIND_BELOW, "tab:blue")
    _triangle(a, above, WIND_ABOVE, "tab:red")
    a.text(0, -PERF.v_max, f"v_max = {PERF.v_max:.0f}", fontsize=8, color="0.5", va="top")
    a.set_aspect("equal", adjustable="box")
    a.set_xlim(-10, 9)
    a.set_ylim(-9, 9)
    a.set_xlabel("East velocity [m/s]")
    a.set_ylabel("North velocity [m/s]")
    a.set_title("Wind triangle: airspeed (colour) + wind (grey) = ground (dotted)")
    a.grid(True, alpha=0.3)

    # --- ground speed vs time ---
    a = ax[1, 0]
    a.plot(_col(below, 0), [math.hypot(r[3], r[4]) for r in below], color="tab:blue",
           lw=2.0, label="below envelope")
    a.plot(_col(above, 0), [math.hypot(r[3], r[4]) for r in above], color="tab:red",
           lw=2.0, label="above envelope")
    a.axhline(GS_CMD, color="0.5", ls=":", lw=1.0, label=f"commanded {GS_CMD:.0f} m/s")
    a.set_xlabel("t [s]")
    a.set_ylabel("ground speed [m/s]")
    a.set_title("Ground speed: exact when feasible, short when clamped")
    a.grid(True, alpha=0.3)
    a.legend(fontsize=8)

    # --- cross-track (east) drift vs time ---
    a = ax[1, 1]
    a.plot(_col(below, 0), _col(below, 1), color="tab:blue", lw=2.0, label="below envelope")
    a.plot(_col(above, 0), _col(above, 1), color="tab:red", lw=2.0, label="above envelope")
    a.axhline(0.0, color="0.5", ls=":", lw=1.0)
    a.set_xlabel("t [s]")
    a.set_ylabel("east (cross-track) drift [m]")
    a.set_title("Cross-track drift: nulled by the crab, or accumulated downwind")
    a.grid(True, alpha=0.3)
    a.legend(fontsize=8)

    fig.suptitle(
        "Phase 5b: a multirotor flies one ground-velocity command in two winds — "
        "the airspeed envelope decides whether the ground track holds",
        fontsize=12,
    )
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


def main() -> None:
    below, above = run(WIND_BELOW), run(WIND_ABOVE)
    for name, rows in (("below (w=4)", below), ("above (w=7)", above)):
        ve, vn = rows[-1][3], rows[-1][4]
        print(f"{name:>12}: ground = ({ve:+.2f}, {vn:+.2f}) m/s, "
              f"speed {math.hypot(ve, vn):.2f} (cmd {GS_CMD:.0f}), "
              f"airspeed {math.hypot(rows[-1][5], rows[-1][6]):.2f} (v_max {PERF.v_max:.0f})")
    img = "vault/observations/img/wind-multirotor-envelope.png"
    out = Path(__file__).resolve().parents[1] / img
    plot(below, above, out)


if __name__ == "__main__":
    main()
