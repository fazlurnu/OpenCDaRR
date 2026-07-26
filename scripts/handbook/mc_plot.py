"""Handbook figures for the Pairwise-conflict environment page.

Runs the Monte-Carlo engine in :mod:`mc_pairwise` (N runs per {scenario} x {resolver} x {recovery}
cell) and draws, per scenario, a 2x2 of ground tracks and a 2x2 of separation-over-time (columns
MVP / VO, rows FTR / Past-CPA), plus the P(LoS) / IPR / CPA summary table. Writes four PNGs into
the site repo (override the directory with ``HANDBOOK_IMG`` for a dry run).

    PYTHONPATH=. python scripts/handbook/mc_plot.py [N]
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mc_pairwise import RECOVERIES, RESOLVERS, run_all  # noqa: E402

IMG = Path(os.environ.get("HANDBOOK_IMG",
                          Path.home() / "Projects/opencdarr.github.io/docs/assets/img"))
RPZ = 50.0
BLUE, ORANGE, RED = "#1f77b4", "#ff7f0e", "#d62728"
RES_COLS = list(RESOLVERS)     # ["MVP", "VO"]
REC_ROWS = list(RECOVERIES)    # ["FTR", "Past-CPA"]
TITLES = {"headon": "Head-on, fixed-wing (waypoint = intruder start)",
          "crossing": "5 deg crossing, fixed-wing (waypoint across)"}
SEP_XLIM = {"headon": (0.0, 150.0), "crossing": (0.0, 320.0)}
TRAJ = {"headon": dict(xlim=(-300, 300), ylim=(-120, 2200)),
        "crossing": dict(xlim=(-450, 450), ylim=(-150, 5200))}


def _cell_title(results, sname, rname, cname):
    _, _, s, _ = results[(sname, rname, cname)]
    return (f"{rname} x {cname}\nP(LoS)={s['p_los']:.2f}  IPR={s['ipr']:.2f}  "
            f"CPA med={s['cpa_med']:.0f} m")


def trajectory_figure(results, sname):
    fig, axes = plt.subplots(2, 2, figsize=(11.0, 11.0), sharex=True, sharey=True)
    for ri, cname in enumerate(REC_ROWS):
        for ci, rname in enumerate(RES_COLS):
            ax = axes[ri, ci]
            runs, _, _, _ = results[(sname, rname, cname)]
            for r in runs:
                for track, col in ((r["tracks"][0], BLUE), (r["tracks"][1], ORANGE)):
                    ax.plot([p[0] for p in track], [p[1] for p in track], color=col, lw=0.6,
                            alpha=0.12)
            ax.plot([], [], color=BLUE, lw=2, label="own")
            ax.plot([], [], color=ORANGE, lw=2, label="intruder")
            ax.set_title(_cell_title(results, sname, rname, cname), fontsize=9)
            ax.set_box_aspect(1)
            ax.set_xlim(*TRAJ[sname]["xlim"])
            ax.set_ylim(*TRAJ[sname]["ylim"])
            if ri == 1:
                ax.set_xlabel("east [m]  (exaggerated)")
            if ci == 0:
                ax.set_ylabel("north [m]")
            if ri == 0 and ci == 0:
                ax.legend(fontsize=8, loc="upper left")
    fig.suptitle(f"{TITLES[sname]} -- {len(runs)} runs, ground tracks", fontsize=12)
    fig.tight_layout()
    out = IMG / f"pairwise-{sname}-tracks.png"
    fig.savefig(out, dpi=125)
    print("wrote", out)


def separation_figure(results, sname):
    fig, axes = plt.subplots(2, 2, figsize=(11.0, 10.5), sharex=True, sharey=True)
    for ri, cname in enumerate(REC_ROWS):
        for ci, rname in enumerate(RES_COLS):
            ax = axes[ri, ci]
            runs, _, _, _ = results[(sname, rname, cname)]
            for r in runs:
                ax.plot(r["ts"], r["seps"], color=RED if r["los"] else "0.35", lw=0.6,
                        alpha=0.5 if r["los"] else 0.12)
            ax.axhline(RPZ, color=RED, ls="--", lw=1.4)
            ax.set_title(_cell_title(results, sname, rname, cname), fontsize=9)
            ax.set_box_aspect(1)
            ax.set_xlim(*SEP_XLIM[sname])
            ax.set_ylim(0, 300)
            if ri == 1:
                ax.set_xlabel("time [s]")
            if ci == 0:
                ax.set_ylabel("separation [m]")
    fig.suptitle(f"{TITLES[sname]} -- {len(runs)} runs, separation (red = LoS)", fontsize=12)
    fig.tight_layout()
    out = IMG / f"pairwise-{sname}-separation.png"
    fig.savefig(out, dpi=125)
    print("wrote", out)


def main() -> None:
    n_runs = int(sys.argv[1]) if len(sys.argv) > 1 else 250
    IMG.mkdir(parents=True, exist_ok=True)
    results = run_all(n_runs)
    for sname in ("headon", "crossing"):
        print(f"\n{TITLES[sname]}")
        for rname in RES_COLS:
            for cname in REC_ROWS:
                _, _, s, _ = results[(sname, rname, cname)]
                print(f"  {rname + ' x ' + cname:16} P(LoS)={s['p_los']:.3f} IPR={s['ipr']:.3f} "
                      f"CPA med={s['cpa_med']:.0f} p5={s['cpa_p5']:.0f} min={s['cpa_min']:.0f}")
        trajectory_figure(results, sname)
        separation_figure(results, sname)


if __name__ == "__main__":
    main()
