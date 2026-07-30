"""Handbook figures: steady wind — how each airframe meets it, and why DAA stays robust.

The "Wind" module page's pictures, all drawn from the real kinematics/loop, with the wind shown as
a background **velocity-vector field** (the direction the air moves):

  1. ``fixedwing`` — a constant-airspeed fixed-wing in a steady turn traces a circle in the air but
     a **trochoid over the ground**, one per wind bearing (the paper's Fig. 4).
  2. ``multirotor`` — a multirotor's envelope is on *airspeed*: with the wind **below** its top
     speed it crabs and holds the commanded ground track; **above** it, the airspeed clamps and the
     track is blown downwind.
  3. ``daa`` — a fixed-wing crossing conflict resolved in still air and in wind: the ground tracks
     crab and bend and the closure shifts, yet MVP + Past-CPA still clear the protected zone.

Handbook plot style: no suptitle, concise titles. Writes into the site repo.

    PYTHONPATH=. python scripts/handbook/wind.py
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.cd import StateBased  # noqa: E402
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.crr import PastCPA  # noqa: E402
from opencdarr.kinematics import FixedWing, MotionCommand, Multirotor  # noqa: E402
from opencdarr.performance import SMALL_FIXEDWING, Performance  # noqa: E402
from opencdarr.scenario import create_conflict  # noqa: E402
from opencdarr.separation import INACTIVE, SeparationManager  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402
from opencdarr.wind import NO_WIND, WindField  # noqa: E402

LAT0, LON0 = 52.0, 4.0
IMG = Path.home() / "Projects/opencdarr.github.io/docs/assets/img"
RPZ, LOOKAHEAD = 50.0, 120.0
BLUE, ORANGE, RED, GREEN, WINDCOL = "#1f77b4", "#ff7f0e", "#d62728", "#2ca02c", "#f2a6a6"
_MR, _FW = Multirotor(), FixedWing()


def _enu(lat: float, lon: float) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(LAT0, LON0, lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def _wind_field(ax: plt.Axes, wind: WindField, xlim: tuple[float, float],
                ylim: tuple[float, float], n: int = 7) -> None:
    """Draw the wind as a faint background velocity-vector field (arrows point downwind) over the
    ``xlim`` × ``ylim`` rectangle; a calm field is drawn as faint dots, as in a no-wind panel."""
    xx, yy = np.meshgrid(np.linspace(*xlim, n), np.linspace(*ylim, n))
    if wind.speed < 1e-9:
        ax.scatter(xx, yy, s=3, color="0.8", zorder=0)
        return
    we, wn = wind.components()
    length = (xlim[1] - xlim[0]) / (n - 1) * 0.6
    u = np.full_like(xx, we / wind.speed * length)
    v = np.full_like(yy, wn / wind.speed * length)
    ax.quiver(xx, yy, u, v, color=WINDCOL, alpha=0.9, scale=1.0, scale_units="xy",
              angles="xy", width=0.007, zorder=0)


# ------------------------------------------------------------------ figure 1: fixed-wing trochoids
def fw_turn(wind: WindField, tmax: float = 23.0,
            dt: float = 0.05) -> tuple[list[float], list[float]]:
    """A fixed-wing in a continuous max-bank turn under ``wind``: the ground track (a trochoid)."""
    s = AircraftState(id="F", lat=LAT0, lon=LON0, trk=0.0, gs=17.0, yaw=0.0, bank=0.0)
    xs, ys = [], []
    t = 0.0
    while t < tmax:
        e, n = _enu(s.lat, s.lon)
        xs.append(e)
        ys.append(n)
        cmd = MotionCommand(target_airspeed_direction=((s.yaw or 0.0) + 90.0) % 360.0,
                            target_airspeed=17.0)
        s = _FW.step(s, cmd, SMALL_FIXEDWING, dt, wind)
        t += dt
    return xs, ys


def fixedwing_figure(out: Path) -> None:
    speed = 5.0  # wind speed [m/s] for every bearing panel
    bearings: list[float | None] = [None, 0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]
    tracks = []
    for b in bearings:
        wind = NO_WIND if b is None else WindField.from_met(b, speed)
        xs, ys = fw_turn(wind)
        cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)  # centre each trochoid in its panel
        tracks.append(([x - cx for x in xs], [y - cy for y in ys]))
    half = max(max(abs(v) for v in xs + ys) for xs, ys in tracks) * 1.12

    fig, axes = plt.subplots(3, 3, figsize=(10.5, 10.5))
    for ax, b, (xs, ys) in zip(axes.flat, bearings, tracks, strict=True):
        wind = NO_WIND if b is None else WindField.from_met(b, speed)
        _wind_field(ax, wind, (-half, half), (-half, half))
        ax.plot(xs, ys, color=BLUE, lw=2.0)
        ax.plot(xs[0], ys[0], "^", color="k", ms=6)
        ax.set_xlim(-half, half)
        ax.set_ylim(-half, half)
        ax.set_aspect("equal")
        ax.set_title("no wind" if b is None else f"wind from {b:.0f}°", fontsize=9)
        ax.tick_params(labelsize=7)
    fig.supxlabel("east [m]", fontsize=9)
    fig.supylabel("north [m]", fontsize=9)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    print("wrote", out)


# ------------------------------------------------------------------ figure 2: multirotor envelope
def mr_track(wind: WindField, perf: Performance, gs_cmd: float = 5.0) -> tuple[list[float],
                                                                              list[float], float]:
    """A multirotor flying a constant ``gs_cmd``-north *ground* command in ``wind``: its ground
    track and the steady-state ground speed (short of the command when the airspeed clamps)."""
    s = AircraftState(id="M", lat=LAT0, lon=LON0, trk=0.0, gs=gs_cmd)
    cmd = MotionCommand.from_track_speed(0.0, gs_cmd)
    xs, ys = [], []
    t = 0.0
    while t < 55.0:
        e, n = _enu(s.lat, s.lon)
        xs.append(e)
        ys.append(n)
        s = _MR.step(s, cmd, perf, 0.1, wind)
        t += 0.1
    return xs, ys, s.gs


def multirotor_figure(out: Path) -> None:
    perf = Performance(v_max=8.0, v_min=-8.0, ax=4.0, yaw_rate_max=90.0)  # top speed 8 m/s
    cases = [("wind 4 m/s < top speed", WindField.from_met(270.0, 4.0)),
             ("wind 10 m/s > top speed", WindField.from_met(270.0, 10.0))]
    runs = [(title, wind, *mr_track(wind, perf)) for title, wind in cases]
    all_x = [v for _, _, xs, _, _ in runs for v in xs]
    all_y = [v for _, _, _, ys, _ in runs for v in ys]
    xlim = (min(all_x) - 25, max(all_x) + 25)
    ylim = (min(all_y) - 15, max(all_y) + 15)
    print(f"     multirotor (top speed {perf.v_max:.0f} m/s, command 5 m/s north):")

    fig, axes = plt.subplots(1, 2, figsize=(10.5, 5.8), sharex=True, sharey=True)
    for ax, (title, wind, xs, ys, gs) in zip(axes, runs, strict=True):
        holds = gs >= 4.99
        _wind_field(ax, wind, xlim, ylim)
        ax.plot(xs, ys, color=(BLUE if holds else RED), lw=2.6,
                label="holds north (crabs)" if holds else "blown downwind (clamped)")
        ax.plot(0, 0, "^", color="k", ms=8)
        ax.axvline(0.0, color="0.8", lw=0.8, zorder=0)
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_xlabel("east [m]")
        ax.set_ylabel("north [m]")
        ax.set_title(f"{title}\nground speed {gs:.1f} m/s (commanded 5)", fontsize=10)
        ax.legend(fontsize=8, loc="upper left")
        print(f"       {title}: ground speed {gs:.2f} m/s")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print("wrote", out)


# ------------------------------------------------------------------ figure 3: DAA under wind
def fw_conflict(wind: WindField) -> tuple[list[float], list[float], list[tuple[float, float]],
                                          list[tuple[float, float]]]:
    """Two fixed-wings resolve a 90° crossing in ``wind`` (StateBased + MVP + Past-CPA, avoidance
    velocity projected to course/airspeed): times, separation, and the two ground tracks. The
    fixed-wing feels the wind — it crabs and its ground speed shifts with heading — so the ground
    tracks bend, yet the stack still opens the miss past the protected zone."""
    from opencdarr.separation import project_to_fixedwing

    own = AircraftState(id="OWN", lat=LAT0, lon=LON0, trk=0.0, gs=17.0, yaw=0.0, bank=0.0)
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=0.0, tlos=60.0, rpz=RPZ, side=1)
    sep = SeparationManager()
    det, res, rec = StateBased(), MVP(margin=1.25), PastCPA(bouncing_guard=True)
    nom_own = MotionCommand.from_track_speed(own.trk, own.gs)
    nom_intr = MotionCommand.from_track_speed(intr.trk, intr.gs)

    def adapt(cmd: MotionCommand) -> MotionCommand:
        return project_to_fixedwing(cmd, SMALL_FIXEDWING)

    mo = mi = INACTIVE
    co, ci = adapt(nom_own), adapt(nom_intr)
    ts, seps, o_xy, i_xy = [], [], [], []
    t, nb = 0.0, 0.0
    while t < 90.0:
        if t + 1e-9 >= nb:
            co, mo = sep.step(own, [intr], nom_own, mo, RPZ, LOOKAHEAD, det, res, rec, adapt)
            ci, mi = sep.step(intr, [own], nom_intr, mi, RPZ, LOOKAHEAD, det, res, rec, adapt)
            nb += 1.0
        _, d = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
        ts.append(t)
        seps.append(d)
        o_xy.append(_enu(own.lat, own.lon))
        i_xy.append(_enu(intr.lat, intr.lon))
        own = _FW.step(own, co, SMALL_FIXEDWING, 0.2, wind)
        intr = _FW.step(intr, ci, SMALL_FIXEDWING, 0.2, wind)
        t += 0.2
    return ts, seps, o_xy, i_xy


def _cpa_mid(o_xy: list[tuple[float, float]], i_xy: list[tuple[float, float]],
             seps: list[float]) -> tuple[float, float]:
    """Midpoint between the two aircraft at closest approach — the centre of the interaction."""
    k = min(range(len(seps)), key=lambda j: seps[j])
    return (o_xy[k][0] + i_xy[k][0]) / 2, (o_xy[k][1] + i_xy[k][1]) / 2


def daa_figure(out: Path) -> None:
    wind = WindField.from_met(225.0, 6.0)
    ts_c, sep_c, o_c, i_c = fw_conflict(NO_WIND)
    ts_w, sep_w, o_w, i_w = fw_conflict(wind)
    print(f"     fixed-wing conflict min-sep: still {min(sep_c):.1f} m, wind {min(sep_w):.1f} m "
          f"(rpz {RPZ:.0f}) — both clear")

    fig, (a_xy, a_sep) = plt.subplots(1, 2, figsize=(11.5, 5.6))

    # --- ground tracks, zoomed to the interaction: still air (faint) vs wind (solid, crabbed) ---
    cx, cy = _cpa_mid(o_w, i_w, sep_w)
    half = 360.0  # window half-width around the closest approach [m]
    xlim, ylim = (cx - half, cx + half), (cy - half, cy + half)
    _wind_field(a_xy, wind, xlim, ylim)
    for o, i, a, lw in ((o_c, i_c, 0.3, 1.4), (o_w, i_w, 1.0, 2.4)):
        a_xy.plot([p[0] for p in o], [p[1] for p in o], color=ORANGE, alpha=a, lw=lw)
        a_xy.plot([p[0] for p in i], [p[1] for p in i], color=BLUE, alpha=a, lw=lw)
    kw = min(range(len(sep_w)), key=lambda j: sep_w[j])
    a_xy.add_patch(plt.Circle((o_w[kw][0], o_w[kw][1]), RPZ, fill=False, ls="--", color=RED,
                              lw=1.0, alpha=0.7))
    a_xy.plot([], [], color="0.4", lw=1.4, alpha=0.5, label="still air")
    a_xy.plot([], [], color="0.4", lw=2.4, label="with wind")
    a_xy.set_xlim(*xlim)
    a_xy.set_ylim(*ylim)
    a_xy.set_aspect("equal")
    a_xy.set_xlabel("east [m]")
    a_xy.set_ylabel("north [m]")
    a_xy.set_title("The fixed-wing tracks crab and bend under wind", fontsize=10)
    a_xy.legend(fontsize=8, loc="upper left")

    # --- separation over time: the miss shifts a little, both clear the protected zone ---
    a_sep.plot(ts_c, sep_c, color="0.45", lw=2.0, label=f"still air ({min(sep_c):.0f} m)")
    a_sep.plot(ts_w, sep_w, color=GREEN, lw=2.0, label=f"with wind ({min(sep_w):.0f} m)")
    a_sep.axhline(RPZ, color=RED, ls=":", lw=1.4, label=f"rpz = {RPZ:.0f} m")
    a_sep.set_ylim(bottom=0)
    a_sep.set_xlabel("time [s]")
    a_sep.set_ylabel("separation [m]")
    a_sep.set_title("Both clear the protected zone", fontsize=10)
    a_sep.legend(fontsize=8, loc="best")
    a_sep.set_box_aspect(1)

    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print("wrote", out)


def main() -> None:
    IMG.mkdir(parents=True, exist_ok=True)
    fixedwing_figure(IMG / "wind-fixedwing.png")
    multirotor_figure(IMG / "wind-multirotor.png")
    daa_figure(IMG / "wind-daa.png")


if __name__ == "__main__":
    main()
