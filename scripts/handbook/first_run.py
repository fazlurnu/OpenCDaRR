"""Handbook figure: a first run — a mixed pair flies to a waypoint, meets, and avoids.

The "A first run" page's picture. A multirotor and a fixed-wing each fly a one-waypoint goto
mission; ``create_conflict`` places them so their straight legs cross a few seconds in, so each
predicts a conflict against the other and manoeuvres (StateBased + MVP + Past-CPA), then both
recover and continue. The same scenario is run twice — once on the true state, once with GNSS
self-noise (:class:`GnssNavigation`) — the one argument the page's two code snippets differ by.

The build (``build_fleet``) and the two runs are exactly what the page shows; the per-tick tracks
and separation come from ``tools.fleet_trace.run_fleet_traced``, which records the same run
:func:`~opencdarr.fleet.run_fleet` scores. Handbook plot style: no grid, no suptitle, square track
axes. Writes into the site repo.

    PYTHONPATH=. python scripts/handbook/first_run.py
"""

from __future__ import annotations

import statistics
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo, rng  # noqa: E402
from opencdarr.autopilot import WaypointAutopilot  # noqa: E402
from opencdarr.cd import StateBased  # noqa: E402
from opencdarr.cns.navigation import GnssNavigation  # noqa: E402
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.crr import PastCPA  # noqa: E402
from opencdarr.dynamics import FixedWing, Multirotor  # noqa: E402
from opencdarr.fleet import Agent, run_fleet  # noqa: E402
from opencdarr.mission import Mission  # noqa: E402
from opencdarr.performance import M600, SMALL_FIXEDWING  # noqa: E402
from opencdarr.scenario import create_conflict  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402
from scripts.handbook.tools.fleet_trace import FleetTrace, enu, run_fleet_traced  # noqa: E402

LAT0, LON0 = 52.0, 4.0
GS_MR, GS_FW = 18.0, 15.0  # the multirotor cruises at v_max; the fixed-wing holds this airspeed
SEED = 20260725
N_RUNS = 100  # Monte Carlo: independent noisy repeats of the same encounter
# the separation stack (detector → resolver → recovery) and the run geometry, shared by both runs
CDR = dict(rpz=50.0, t_lookahead=20.0, dt=0.5, detector=StateBased(),
           resolver=MVP(margin=1.1), recovery=PastCPA(bouncing_guard=True))
NOISE = GnssNavigation()


def build_fleet(noisy: bool) -> tuple[list[Agent], list[tuple[float, float]]]:
    """The mixed pair and their waypoints. The multirotor cruises north; ``create_conflict`` places
    the fixed-wing so their legs lose separation 30 s out — later than the 20 s look-ahead, so the
    conflict only appears a few seconds in. Each waypoint sits far down its own initial track, so
    the nominal path runs straight through the crossing. ``noisy`` sets the GNSS accuracy."""
    ci, vci = (15.0, 1.5) if noisy else (0.0, 0.0)
    copter = AircraftState(id="COPTER", lat=LAT0, lon=LON0, trk=0.0, gs=GS_MR, yaw=0.0,
                           pos_ci95=ci, vel_ci95=vci)
    plane = create_conflict(copter, intr_id="PLANE", dpsi=90.0, dcpa=0.0, tlos=30.0, rpz=50.0,
                            gs_intr=GS_FW, side=1)
    wp_copter = geo.forward(copter.lat, copter.lon, copter.trk, 800.0)
    wp_plane = geo.forward(plane.lat, plane.lon, plane.trk, 800.0)
    agents = [
        Agent(copter, M600, Multirotor(), WaypointAutopilot(Mission(goto=wp_copter))),
        Agent(plane, SMALL_FIXEDWING, FixedWing(),
              WaypointAutopilot(Mission(goto=wp_plane), cruise_airspeed=GS_FW)),
    ]
    return agents, [wp_copter, wp_plane]


def plot(clean: FleetTrace, noisy: FleetTrace, wps: list[tuple[float, float]], out: Path) -> None:
    fig, (ax_xy, ax_sep) = plt.subplots(1, 2, figsize=(12.5, 5.6))
    origin = (LAT0, LON0)
    blue, orange = "#1f77b4", "#ff7f0e"

    # --- ground tracks: two aircraft, solid = true-state run, dashed = GNSS-noise run ---
    for tr, ls, lw, a in ((clean, "-", 2.4, 1.0), (noisy, "--", 1.6, 0.9)):
        for i, col in ((0, blue), (1, orange)):
            xs = [p[0] for p in tr.tracks[i]]
            ys = [p[1] for p in tr.tracks[i]]
            ax_xy.plot(xs, ys, color=col, ls=ls, lw=lw, alpha=a)
    all_pts: list[tuple[float, float]] = []
    for i, (col, name) in enumerate(((blue, "multirotor"), (orange, "fixed-wing"))):
        x0, y0 = clean.tracks[i][0]
        ax_xy.plot(x0, y0, "^", color=col, ms=10, label=f"{name} start")
        wx, wy = enu(origin, *wps[i])
        ax_xy.plot(wx, wy, "*", color=col, ms=15)
        all_pts += [(x0, y0), (wx, wy)] + clean.tracks[i] + noisy.tracks[i]
    # closest approach on the true-state run (the low point of the separation panel)
    k = min(range(len(clean.min_sep)), key=lambda j: clean.min_sep[j])
    for i, col in ((0, blue), (1, orange)):
        ax_xy.plot(*clean.tracks[i][k], "o", color=col, ms=5, zorder=6)
    ax_xy.plot([], [], color="0.4", ls="-", lw=2.0, label="true state")
    ax_xy.plot([], [], color="0.4", ls="--", lw=1.6, label="with GNSS noise")
    # a square box tight around the tracks (no forced symmetry — the tracks fill the panel)
    xs_, ys_ = [p[0] for p in all_pts], [p[1] for p in all_pts]
    cx, cy = (min(xs_) + max(xs_)) / 2, (min(ys_) + max(ys_)) / 2
    half = max(max(xs_) - min(xs_), max(ys_) - min(ys_)) / 2 * 1.08
    ax_xy.set_xlim(cx - half, cx + half)
    ax_xy.set_ylim(cy - half, cy + half)
    ax_xy.set_aspect("equal")
    ax_xy.set_xlabel("east [m]")
    ax_xy.set_ylabel("north [m]")
    ax_xy.set_title("ground tracks (▲ start, ★ waypoint, ● closest approach)", fontsize=10)
    ax_xy.legend(fontsize=8, loc="lower right")

    # --- separation over time: closes, conflict is detected, the manoeuvre holds it above rpz ---
    for tr, ls, lw, lab in ((clean, "-", 2.2, "true state"),
                            (noisy, "--", 1.8, "with GNSS noise")):
        ax_sep.plot(tr.t, tr.min_sep, color="#2ca02c", ls=ls, lw=lw, label=lab)
    ax_sep.axhline(CDR["rpz"], color="#d62728", ls=":", lw=1.4, label=f"rpz = {CDR['rpz']:.0f} m")
    t_res = clean.first_resolving()
    if t_res is not None:
        ax_sep.axvline(t_res, color="0.6", lw=1.0)
        ax_sep.annotate("conflict detected → avoid", (t_res, ax_sep.get_ylim()[1]),
                        textcoords="offset points", xytext=(5, -12), fontsize=8, color="0.35")
    ax_sep.set_ylim(bottom=0)
    ax_sep.set_xlabel("time [s]")
    ax_sep.set_ylabel("separation [m]")
    ax_sep.set_title("pairwise separation — the manoeuvre holds it above rpz", fontsize=10)
    ax_sep.legend(fontsize=8, loc="best")
    ax_sep.set_box_aspect(1)

    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    print("wrote", out)


def montecarlo(clean_sep: float, out: Path) -> None:
    """Run the same noisy encounter ``N_RUNS`` times, each with its own RNG substream (all spawned
    from one root seed, ADR 0001), and draw where the closest approach lands. One noisy run is a
    single sample; the aggregate is the safety statement — the loss-of-separation rate and how the
    margin to the protected zone is really distributed."""
    agents = build_fleet(noisy=True)[0]
    outcomes = [run_fleet(agents, navigation=NOISE, rng=rng.generator(seq), **CDR)
                for seq in rng.spawn(rng.root_seed_sequence(SEED), N_RUNS)]
    sep = [o.min_sep for o in outcomes]
    los = sum(o.los for o in outcomes)
    worst, median = min(sep), statistics.median(sep)
    print(f"{'monte carlo':>16}: {los}/{N_RUNS} lost separation | "
          f"min {worst:.1f} m, median {median:.1f} m, max {max(sep):.1f} m")

    rpz = CDR["rpz"]
    fig, ax = plt.subplots(figsize=(7.6, 5.0))
    ax.hist(sep, bins=range(50, int(max(sep)) + 10, 5), color="#2ca02c", alpha=0.8,
            edgecolor="white")
    ax.axvline(rpz, color="#d62728", ls="--", lw=1.6, label=f"protected zone (rpz = {rpz:.0f} m)")
    ax.axvline(clean_sep, color="0.35", ls=":", lw=1.6,
               label=f"single clean run ({clean_sep:.0f} m)")
    ax.set_xlabel("closest approach over the run [m]")
    ax.set_ylabel(f"runs (of {N_RUNS})")
    ax.set_title(f"Where the closest approach lands over {N_RUNS} noisy repeats", fontsize=10)
    ax.legend(fontsize=8, loc="upper right")
    ax.annotate(f"{los} / {N_RUNS} lost separation\nworst {worst:.1f} m · median {median:.1f} m",
                (0.97, 0.62), xycoords="axes fraction", ha="right", va="top", fontsize=9,
                bbox=dict(boxstyle="round,pad=0.4", fc="white", ec="0.7"))

    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    print("wrote", out)


def main() -> None:
    clean = run_fleet_traced(build_fleet(noisy=False)[0], **CDR)
    noisy = run_fleet_traced(build_fleet(noisy=True)[0], navigation=NOISE,
                             rng=rng.generator(rng.root_seed_sequence(SEED)), **CDR)
    for label, tr in (("true state", clean), ("with GNSS noise", noisy)):
        detected = tr.first_resolving()
        print(f"{label:>16}: min_sep {tr.worst_sep:.1f} m | "
              f"conflict detected at {f'{detected:.1f}s' if detected is not None else 'never'} | "
              f"{'clear' if tr.worst_sep >= CDR['rpz'] else 'LOSS'}")
    img = Path.home() / "Projects/opencdarr.github.io/docs/assets/img"
    plot(clean, noisy, build_fleet(noisy=False)[1], img / "first-run.png")
    montecarlo(clean.worst_sep, img / "first-run-montecarlo.png")


if __name__ == "__main__":
    main()
