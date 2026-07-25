"""Phase-6 diagnostic: the MVP resolution-bias floor ``_BIAS_EPS``, before and after.

MVP's head-on exception (``opencdarr/cr/mvp.py``) floors the predicted miss and picks a clean
perpendicular side when a conflict is near-head-on — the same handler BlueSky's MVP has (``mvp.py``
line 299), but BlueSky floors at **10 m** while our re-derivation used **1e-3 m**. Below ~0.1 m of
miss the *actual* CPA-offset direction is noise-dominated, so the resolution came out
ill-conditioned and a near-head-on pair would drag each other off course (a livelock). This renders
same two cooperative fleets (3-aircraft ±60°, and the 8-aircraft ring, both flying goto missions to
their waypoints) at ``_BIAS_EPS = 1e-3`` (broken) and ``0.1`` (fixed).

Writes ``vault/observations/img/headon-threshold-comparison.png``.

    PYTHONPATH=. python scripts/headon_threshold_demo.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import opencdarr.cr.mvp as mvpmod  # noqa: E402
import scripts.fleet_cooperative_demo as d8  # noqa: E402
import scripts.three_aircraft_demo as d3  # noqa: E402
from opencdarr.cr import MVP  # noqa: E402

_EPS_VALUES = (1e-3, 0.1)


def _run3(eps: float) -> tuple[list, list, float, float]:
    mvpmod._BIAS_EPS = eps
    tracks, pair_sep, times, wps = d3.simulate(MVP(margin=d3.MARGIN))
    return tracks, wps, min(min(s) for s in pair_sep), times[-1]


def _run8(eps: float) -> tuple[list, list, float, float]:
    mvpmod._BIAS_EPS = eps
    tracks, min_sep = d8.simulate(MVP(margin=d8.MARGIN))
    _, wps = d8._starts_and_waypoints()
    return tracks, [d8._enu(w[0], w[1]) for w in wps], min(min_sep), len(min_sep) * d8.DT


def _draw(ax: plt.Axes, tracks: list, wps: list, colors: list, lim: float,
          eps: float, min_sep: float, secs: float) -> None:
    for k, trk in enumerate(tracks):
        ax.plot([p[0] for p in trk], [p[1] for p in trk], color=colors[k], lw=2.0)
        ax.scatter([trk[0][0]], [trk[0][1]], color=colors[k], marker="^", s=40, zorder=5)
        ax.scatter([wps[k][0]], [wps[k][1]], color=colors[k], marker="*", s=110, zorder=5)
    ax.scatter([0], [0], color="k", marker="x", s=45, zorder=6)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    verdict = "clear" if min_sep >= 50.0 else "LOSS"
    ax.set_title(f"_BIAS_EPS = {eps:g}\nmin sep {min_sep:.0f} m ({verdict}), {secs:.0f} s",
                 fontsize=10)
    ax.grid(True, alpha=0.3)


def main() -> None:
    cmap = plt.get_cmap("hsv")
    c3 = list(d3._COLORS)
    c8 = [cmap(k / d8.N) for k in range(d8.N)]
    fig, ax = plt.subplots(2, 2, figsize=(12.0, 12.0))

    for col, eps in enumerate(_EPS_VALUES):
        t3, w3, m3, s3 = _run3(eps)
        _draw(ax[0, col], t3, w3, c3, 3000.0, eps, m3, s3)
        t8, w8, m8, s8 = _run8(eps)
        _draw(ax[1, col], t8, w8, c8, d8.RADIUS * 1.15, eps, m8, s8)
        print(f"eps={eps:g}: 3-ac min={m3:.1f}m/{s3:.0f}s   8-ac min={m8:.1f}m/{s8:.0f}s")
    ax[0, 0].set_ylabel("3-aircraft ±60°\nNorth [m]")
    ax[1, 0].set_ylabel("8-aircraft ring\nNorth [m]")
    for a in (ax[1, 0], ax[1, 1]):
        a.set_xlabel("East [m]")

    fig.suptitle(
        "MVP head-on threshold: 1e-3 (ill-conditioned near-head-on -> drag/livelock) vs 0.1 "
        "(clean). Same cooperative fleets flying goto missions; ▲ start, ★ waypoint.",
        fontsize=12,
    )
    fig.tight_layout()
    img = "vault/observations/img/headon-threshold-comparison.png"
    out = Path(__file__).resolve().parents[1] / img
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")
    mvpmod._BIAS_EPS = 0.1  # leave the runtime at the adopted value


if __name__ == "__main__":
    main()
