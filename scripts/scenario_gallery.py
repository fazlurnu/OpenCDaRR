"""Figures for ``opencdarr/scenario/README.md``.

Two of them:

- ``docs/img/scenario-gallery.png`` — every shipped geometry, one panel each. The traffic panel
  shows both circles: aircraft are released on the dotted one and measured inside the dashed one.
- ``docs/img/scenario-ring-variants.png`` — ``swap_ring`` against ``crossing_ring`` at n = 3, 4, 5,
  which is where the two stop being the same fleet.

    PYTHONPATH=. python scripts/scenario_gallery.py
"""

from __future__ import annotations

import math
import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.scenario import (  # noqa: E402
    PairwiseEncounter,
    converging_ring,
    crossing_ring,
    near_parallel,
    random_traffic,
    swap_pair,
    swap_ring,
)
from opencdarr.scenario.random_traffic import MEASURED_FRACTION  # noqa: E402

LAT0, LON0 = 52.0, 4.0
OUT = pathlib.Path("docs/img")


def enu(lat: float, lon: float) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(LAT0, LON0, lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def draw_fleet(ax, fleet, *, colour="tab:blue", length=600.0) -> None:
    """Each aircraft as a start marker plus its route, drawn at most ``length`` metres.

    Capped because an arrow whose far end lies outside the axes is clipped away entirely, which
    would silently leave a panel showing only its start markers.
    """
    for state, goal in fleet:
        x0, y0 = enu(state.lat, state.lon)
        if goal is None:
            x1, y1 = enu(*geo.forward(state.lat, state.lon, state.trk, length))
        else:
            gx, gy = enu(*goal)
            span = math.hypot(gx - x0, gy - y0)
            frac = min(1.0, length / span) if span > 0 else 0.0
            x1, y1 = x0 + (gx - x0) * frac, y0 + (gy - y0) * frac
        ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                    arrowprops=dict(arrowstyle="->", color=colour, lw=1.2, alpha=0.85))
        ax.plot(x0, y0, marker="o", ms=3.5, color=colour)


def tidy(ax, title: str, limit: float) -> None:
    ax.set_title(title, fontsize=9)
    ax.set_aspect("equal")
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_xticks([])
    ax.set_yticks([])


def gallery() -> None:
    """One panel per shipped geometry, grouped by family."""
    fig, axes = plt.subplots(2, 4, figsize=(11.0, 5.8))
    cfg = _config()

    # --- the two-aircraft family ---
    tidy(axes[0][0], "swap_pair", 1900.0)
    draw_fleet(axes[0][0], swap_pair(span=3000.0), length=3200.0)

    tidy(axes[0][1], "near_parallel (dpsi = 5°)", 1300.0)
    draw_fleet(axes[0][1], near_parallel(reach=3000.0), length=1100.0)

    tidy(axes[0][2], "PairwiseEncounter — 12 draws", 1400.0)
    for k in range(12):
        draw_fleet(axes[0][2], PairwiseEncounter().draw(np.random.default_rng(k), cfg),
                   colour="tab:purple", length=900.0)

    tidy(axes[0][3], "PairwiseEncounter — dpsi = 90°", 1400.0)
    for k in range(12):
        draw_fleet(axes[0][3],
                   PairwiseEncounter(dpsi=90.0).draw(np.random.default_rng(k), cfg),
                   colour="tab:purple", length=900.0)

    # --- the ring family ---
    for ax, (name, build) in zip(
        axes[1][:3],
        (("swap_ring(8)", swap_ring), ("crossing_ring(8)", crossing_ring),
         ("converging_ring(8)", converging_ring)),
        strict=True,
    ):
        tidy(ax, name, 1900.0)
        draw_fleet(ax, build(8, radius=1500.0), colour="tab:red", length=3200.0)

    # --- traffic, with the two areas that come with it ---
    ax = axes[1][3]
    radius = 1500.0
    tidy(ax, "random_traffic — released on the ring", 1900.0)
    draw_fleet(ax, random_traffic(np.random.default_rng(4), 22, radius=radius),
               colour="tab:green", length=800.0)
    ax.add_patch(plt.Circle((0, 0), radius, fill=False, color="0.45", lw=1.0, ls=":"))
    ax.add_patch(plt.Circle((0, 0), radius * MEASURED_FRACTION, fill=False, color="0.2",
                            lw=1.2, ls="--"))

    fig.tight_layout()
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / "scenario-gallery.png", dpi=140, bbox_inches="tight")
    print(f"wrote {OUT / 'scenario-gallery.png'}")


def ring_variants() -> None:
    """Where swap_ring and crossing_ring stop agreeing: odd fleet sizes."""
    radius = 1500.0
    fig, axes = plt.subplots(2, 3, figsize=(9.0, 6.2))

    for col, n in enumerate((3, 4, 5)):
        for row, (name, build, colour) in enumerate(
            (("swap_ring", swap_ring, "tab:orange"), ("crossing_ring", crossing_ring, "tab:blue"))
        ):
            ax = axes[row][col]
            fleet = build(n, radius=radius)
            draw_fleet(ax, fleet, colour=colour, length=3200.0)
            ax.add_patch(plt.Circle((0, 0), radius, fill=False, color="0.7", lw=0.9, ls=":"))
            ax.plot(0, 0, marker="+", ms=9, color="k", mew=1.4)

            miss = min(_miss(state, goal) for state, goal in fleet)
            tidy(ax, f"{name}(n = {n}) — closest route {miss:,.0f} m", 1750.0)

    fig.tight_layout()
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / "scenario-ring-variants.png", dpi=140, bbox_inches="tight")
    print(f"wrote {OUT / 'scenario-ring-variants.png'}")


def _miss(state, goal) -> float:
    """How far the ring centre is from the straight route this aircraft flies [m]."""
    x1, y1 = enu(state.lat, state.lon)
    x2, y2 = enu(*goal)
    return abs(x1 * y2 - x2 * y1) / math.hypot(x2 - x1, y2 - y1)


def _config():
    from opencdarr.config import (
        Config,
        ConflictConfig,
        MethodsConfig,
        ScenarioConfig,
        SimulationConfig,
    )
    return Config(
        seed=0, n_encounters=1,
        scenario=ScenarioConfig("M600", 10.2889, 200.0, 90.0),
        conflict=ConflictConfig(50.0, 120.0),
        methods=MethodsConfig("statebased", "mvp", "pastcpa", 1.05, False),
        simulation=SimulationConfig(1.0, 300.0, 10.0),
    )


if __name__ == "__main__":
    gallery()
    ring_variants()
