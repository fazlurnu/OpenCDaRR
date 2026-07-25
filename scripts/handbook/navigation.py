"""Handbook figure: navigation noise along a trajectory.

The "Navigation" module page's picture — one aircraft flies a straight,
constant-speed leg while :class:`~opencdarr.cns.GnssNavigation` measures its true
state at each broadcast tick. Each dot is a **broadcast fix**: what a receiver
would actually see for that tick. Two panels share the same ``pos_ci95`` but use
different position-error distributions, so the difference is purely in the tails:

  * ``gaussian`` — jitters evenly around the true path;
  * ``make_mixture_gaussian`` — hugs the path most of the time but throws the
    occasional large outlier.

Handbook plot style: no suptitle, concise subplot titles. Writes into the site repo.

    PYTHONPATH=. python scripts/handbook/navigation.py
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.cns import GnssNavigation, gaussian, make_mixture_gaussian  # noqa: E402
from opencdarr.rng import generator, root_seed_sequence, spawn  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

LAT0, LON0 = 52.0, 4.0
POS_CI95 = 40.0  # m — 95% radial position accuracy
CADENCE = 2.0  # s between broadcast fixes
GS, TRK = 10.0, 45.0  # m/s, deg — straight constant-speed leg
N_FIX = 25  # number of broadcast fixes along the leg
SEED = 20260725
OUT = Path.home() / "Projects/opencdarr.github.io/docs/assets/noisy-trajectory.png"

PANELS = (
    ("Gaussian (isotropic)", gaussian, "#4C72B0"),
    ("Mixture (heavy tail)", make_mixture_gaussian(tail_ratio=3.0, tail_weight=0.1), "#C44E52"),
)


def _enu(lat: float, lon: float) -> tuple[float, float]:
    """(east, north) [m] of a lat/lon relative to the leg's origin."""
    qdr, dist = geo.qdrdist(LAT0, LON0, lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def _true_leg() -> list[AircraftState]:
    """The true state at each broadcast tick along a straight constant-speed leg."""
    ve, vn = GS * math.sin(math.radians(TRK)), GS * math.cos(math.radians(TRK))
    states = []
    for k in range(N_FIX):
        east, north = ve * CADENCE * k, vn * CADENCE * k
        bearing = math.degrees(math.atan2(east, north)) % 360.0
        lat, lon = geo.forward(LAT0, LON0, bearing, math.hypot(east, north))
        states.append(
            AircraftState(id="OWN", lat=lat, lon=lon, trk=TRK, gs=GS, pos_ci95=POS_CI95)
        )
    return states


def main() -> None:
    truth = _true_leg()
    true_xy = np.array([_enu(s.lat, s.lon) for s in truth])

    root = root_seed_sequence(SEED)
    fig, axes = plt.subplots(1, 2, figsize=(10, 5), sharex=True, sharey=True)

    for ax, (title, dist, color), seed in zip(axes, PANELS, spawn(root, len(PANELS))):
        nav = GnssNavigation(pos_distribution=dist)
        rng = generator(seed)
        fixes = np.array([_enu(*_fix(nav, s, rng)) for s in truth])

        ax.plot(true_xy[:, 0], true_xy[:, 1], color="0.15", lw=2.2, label="true path")
        ax.plot(fixes[:, 0], fixes[:, 1], color=color, lw=0.6, alpha=0.5, zorder=1)
        ax.scatter(fixes[:, 0], fixes[:, 1], s=14, color=color, zorder=2, label="broadcast fix")
        ax.set_title(title, fontsize=10)
        ax.set_aspect("equal")
        ax.set_xlabel("East [m]")
    axes[0].set_ylabel("North [m]")
    axes[0].legend(loc="upper left", fontsize=9, framealpha=0.9)

    fig.tight_layout()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=130)
    print(f"wrote {OUT}")


def _fix(nav: GnssNavigation, true: AircraftState, rng: np.random.Generator) -> tuple[float, float]:
    """The (lat, lon) a receiver sees for this tick's broadcast fix."""
    m = nav.measure(true, t=0.0, rng=rng).state
    return m.lat, m.lon


if __name__ == "__main__":
    main()
