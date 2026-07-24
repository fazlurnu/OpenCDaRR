"""Visualise the pluggable position-error distributions.

Draws many (east, north) samples from each :class:`~opencdarr.cns.NoiseDistribution`
and shows two things:

* a scatter per distribution with the 95% radial containment circle (``ci95``) —
  all four are calibrated so ~95% of samples fall inside it;
* the empirical radial CDFs together, confirming every curve crosses 0.95 at
  ``ci95`` while differing in shape (heavy tail lifts the mid-range; anisotropy is
  invisible radially — it only reshapes the scatter).

Usage:  python scripts/noise_distributions_demo.py

Writes ``vault/observations/img/noise-distributions.png``.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from opencdarr.cns import (  # noqa: E402
    gaussian,
    make_anisotropic_gaussian,
    make_anisotropic_mixture_gaussian,
    make_mixture_gaussian,
)
from opencdarr.rng import generator, root_seed_sequence, spawn  # noqa: E402

CI95 = 50.0  # m — 95% radial position accuracy
N = 4000  # samples per distribution
SEED = 20260724
OUT = Path("vault/observations/img/noise-distributions.png")

DISTRIBUTIONS = {
    "Gaussian\n(isotropic)": gaussian,
    "Mixture\n(heavy tail)": make_mixture_gaussian(tail_ratio=3.0, tail_weight=0.1),
    "Anisotropic\n(var_ratio 3)": make_anisotropic_gaussian(var_ratio=3.0),
    "Anisotropic mixture\n(both)": make_anisotropic_mixture_gaussian(
        var_ratio=3.0, tail_ratio=3.0, tail_weight=0.1
    ),
}
COLORS = ["#4C72B0", "#C44E52", "#55A868", "#8172B3"]


def _sample(dist, rng: np.random.Generator) -> np.ndarray:
    """``(N, 2)`` array of (east, north) errors [m]."""
    return np.array([dist(rng, CI95) for _ in range(N)])


def main() -> None:
    root = root_seed_sequence(SEED)
    samples = [_sample(d, generator(s)) for d, s in zip(DISTRIBUTIONS.values(), spawn(root, 4))]

    lim = CI95 * 3.2
    theta = np.linspace(0.0, 2.0 * np.pi, 200)
    circle = CI95 * np.cos(theta), CI95 * np.sin(theta)

    fig = plt.figure(figsize=(13, 7))
    gs = fig.add_gridspec(2, 4, height_ratios=[1.35, 1.0], hspace=0.35, wspace=0.25)

    # top row: one scatter + ci95 circle per distribution
    for col, (name, s, color) in enumerate(zip(DISTRIBUTIONS, samples, COLORS)):
        ax = fig.add_subplot(gs[0, col])
        ax.scatter(s[:, 0], s[:, 1], s=4, alpha=0.25, color=color, edgecolors="none")
        ax.plot(*circle, color="0.25", lw=1.3, ls="--")
        ax.set_title(name, fontsize=10)
        ax.set_aspect("equal")
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.axhline(0.0, color="0.85", lw=0.6, zorder=0)
        ax.axvline(0.0, color="0.85", lw=0.6, zorder=0)
        if col == 0:
            ax.set_ylabel("North error [m]")
        ax.set_xlabel("East error [m]")

    # bottom row: radial CDFs overlaid
    axc = fig.add_subplot(gs[1, :])
    for (name, _), s, color in zip(DISTRIBUTIONS.items(), samples, COLORS):
        radial = np.sort(np.hypot(s[:, 0], s[:, 1]))
        cdf = np.arange(1, N + 1) / N
        axc.plot(radial, cdf, color=color, lw=1.8, label=name.replace("\n", " "))
    axc.axhline(0.95, color="0.5", lw=0.9, ls=":")
    axc.axvline(CI95, color="0.5", lw=0.9, ls=":")
    axc.annotate(
        f"95% @ ci95 = {CI95:.0f} m",
        xy=(CI95, 0.95), xytext=(CI95 * 1.15, 0.6),
        arrowprops=dict(arrowstyle="->", color="0.5"), fontsize=9, color="0.3",
    )
    axc.set_xlim(0.0, lim)
    axc.set_ylim(0.0, 1.0)
    axc.set_xlabel("Radial error [m]")
    axc.set_ylabel("Empirical CDF")
    axc.set_title("Radial containment — all four calibrated to the same 95% CI", fontsize=10)
    axc.legend(loc="lower right", fontsize=9, framealpha=0.9)
    axc.grid(True, alpha=0.3)

    fig.suptitle(
        f"Position-error distributions ({N} samples each, ci95 = {CI95:.0f} m)",
        fontsize=13, y=0.98,
    )
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=150, bbox_inches="tight")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
