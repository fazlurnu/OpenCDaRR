"""The §2 geometry, drawn — d_CPA, alpha, gamma, theta and epsilon in one picture.

Everything §2 derives lives in one right-angle construction, and the construction explains
`epsilon` better than the algebra does. Working in the **relative frame**: the ownship sits at the
origin `O` carrying the protected zone, and the intruder at `P` slides down the relative-velocity
ray. A conflict is that ray passing within `rpz` of `O`.

- `alpha` is the angle at `P` between the line of sight `P -> O` and the ray.
- `gamma` is the angle at `P` of the ray that grazes the protected zone, `asin(rpz / |x_rel|)`.
- `theta = gamma - alpha` is the rotation MVP applies, and it lands the ray exactly on the tangent.
- `d_CPA` is the perpendicular from `O` to the ray.
- `epsilon = cos(theta)` is the ratio that makes that landing exact: the rotated ray crosses the
  original `d_CPA` line at `rpz / epsilon`, not at `rpz`, because the tangent point sits
  further along.
  Aiming at `rpz` on that line instead would leave the ray grazing *inside* the zone.

Panel (b) is the same construction with RMVP's rotation, which asks for `k * sigma_m` of angle
beyond the tangent and so grazes a larger circle.

    python robust-mvp/fig_geometry.py
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Arc, Circle  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from rmvp import angular_sigma, rotation  # noqa: E402

GREY, RED, BLUE, ORANGE, PURPLE = "0.45", "#d62728", "#1f77b4", "#ff7f0e", "#9467bd"

# Drawn for legibility, not from the campaign: at Delta-psi = 2 deg the campaign's own gamma is
# 27.3 deg but RMVP's rotation is 83 deg, and at 30 deg gamma is 3.0 deg. Neither renders. The
# angles below are representative; the caption carries the real ones.
RANGE = 100.0     # m, |x_rel|
RPZ = 50.0        # m
D_MISS = 25.0     # m, |d_CPA|
SIGMA_R = 5.7772  # m, the study's relative position sigma
SIGMA_V = 1.7332  # m/s, the study's relative velocity sigma
V_REL = 4.0       # m/s, chosen so the fixed point separates visibly from the one-shot
K = 1.6448536269514722  # Phi^-1(0.95)


def ray(p_start: np.ndarray, angle: float, length: float) -> np.ndarray:
    """Points along the ray leaving ``p_start`` at ``angle`` from the inbound line of sight."""
    direction = np.array([-math.cos(angle), math.sin(angle)])
    return np.array([p_start, p_start + length * direction])


def draw(ax: plt.Axes, alpha: float, gamma: float, theta: float, *, rmvp: bool,
         one_shot: float | None = None) -> None:
    """One panel of the construction.

    ``theta`` is the rotation actually applied. ``one_shot`` is the rotation a rule that
    evaluated ``sigma_m`` once at ``theta = 0`` would have demanded — drawn only where it
    differs from ``theta``, which is exactly where the fixed point is doing work.
    """
    origin = np.array([0.0, 0.0])
    p = np.array([RANGE, 0.0])
    reach = RANGE * 1.35

    ax.add_patch(Circle(origin, RPZ, fill=False, ls="--", lw=1.2, color="k", zorder=2))
    ax.plot(*origin, "o", ms=7, color="k", zorder=5)
    ax.plot(*p, "o", ms=7, color="k", zorder=5)
    ax.annotate("O  ownship", origin, textcoords="offset points", xytext=(-14, -16), fontsize=8)
    ax.annotate("P  intruder", p, textcoords="offset points", xytext=(-16, 10), fontsize=8)
    ax.plot([0, RANGE], [0, 0], color="0.7", lw=1.0, zorder=1)  # line of sight

    # the current relative-velocity ray, and the perpendicular from O onto it
    ax.plot(*ray(p, alpha, reach).T, color=GREY, lw=1.8, zorder=3)
    c = np.array([RANGE * math.sin(alpha) ** 2, RANGE * math.sin(alpha) * math.cos(alpha)])
    ax.annotate("", xy=c, xytext=origin, zorder=4,
                arrowprops={"arrowstyle": "-|>", "color": BLUE, "lw": 2.0})
    ax.annotate(r"$\mathbf{d}_{CPA}$", c * 0.55, textcoords="offset points", xytext=(-46, -2),
                fontsize=10, color=BLUE)
    # right-angle tick at C: d_CPA perpendicular to the ray, the fact 2.2 proves
    along = np.array([-math.cos(alpha), math.sin(alpha)]) * 5.0
    perp = c / np.linalg.norm(c) * 5.0
    ax.plot(*np.array([c - along, c - along - perp, c - perp]).T, color=BLUE, lw=0.9, zorder=4)

    # the rotated ray: MVP stops on the tangent (theta = gamma - alpha), RMVP goes past it
    end = alpha + theta
    ax.plot(*ray(p, end, reach).T, color=RED if rmvp else ORANGE, lw=1.8, zorder=3)
    achieved = RANGE * math.sin(end)
    if rmvp:  # the larger circle the rotated ray actually grazes
        ax.add_patch(Circle(origin, achieved, fill=False, ls=":", lw=1.2, color=RED, zorder=2))
        ax.plot(*ray(p, gamma, reach).T, color=ORANGE, lw=1.2, ls="--", zorder=3, alpha=0.8)
        if one_shot is not None:
            ax.plot(*ray(p, alpha + one_shot, reach).T, color=PURPLE, lw=1.2, ls=(0, (1, 2)),
                    zorder=3)
            same = abs(one_shot - theta) < 1e-3
            tag = "one-shot =\nfixed point" if same else "one-shot"
            ax.annotate(tag, ray(p, alpha + one_shot, reach * (0.85 if same else 0.62))[1],
                        textcoords="offset points", xytext=(-6, 4), fontsize=7.5, color=PURPLE,
                        ha="right")

    # where the rotated ray crosses the d_CPA line: |OC'| = rpz / cos(theta) for MVP
    los_perp = np.array([math.sin(alpha), math.cos(alpha)])
    c_prime = los_perp * (RANGE * math.sin(end) / math.cos(theta))
    tangent_pt = np.array([RANGE * math.sin(end) ** 2, RANGE * math.sin(end) * math.cos(end)])
    if not rmvp:
        ax.plot([c[0], c_prime[0]], [c[1], c_prime[1]], color=BLUE, lw=1.2, ls=":", zorder=3)
        ax.plot(*c_prime, "s", ms=5, color=BLUE, zorder=5)
    ax.plot(*tangent_pt, "o", ms=5, color=RED if rmvp else ORANGE, zorder=5)
    ax.annotate("", xy=tangent_pt, xytext=origin, zorder=4,
                arrowprops={"arrowstyle": "-|>", "color": RED if rmvp else ORANGE, "lw": 1.4})

    # Angle arcs, all at P and all measured from the inbound line of sight. Matplotlib's Arc
    # with angle=180 puts local 0 on the -x axis, so a local sweep [-v, 0] is the global wedge
    # from (180 - v) to 180 degrees, and the arc's midpoint is at global (180 - v/2).
    def arc(value_lo: float, value_hi: float, radius: float, colour: str, label: str) -> None:
        ax.add_patch(Arc(p, 2 * radius, 2 * radius, angle=180.0,
                         theta1=-math.degrees(value_hi), theta2=-math.degrees(value_lo),
                         color=colour, lw=1.7, zorder=4))
        mid = math.pi - 0.5 * (value_lo + value_hi)
        ax.annotate(label, p + radius * 1.14 * np.array([math.cos(mid), math.sin(mid)]),
                    fontsize=12, color=colour, ha="center", va="center", zorder=6)

    arc(0.0, alpha, 34.0, GREY, r"$\alpha$")
    if rmvp:
        arc(0.0, gamma, 52.0, ORANGE, r"$\gamma$")
        arc(gamma, end, 88.0, RED, r"$k\,\sigma_m$")
    else:
        arc(alpha, end, 52.0, RED, r"$\theta$")
        arc(0.0, gamma, 72.0, ORANGE, r"$\gamma$")

    if not rmvp:
        ax.annotate(r"$R_{PZ}$", tangent_pt * 0.62, textcoords="offset points", xytext=(-34, 6),
                    fontsize=10, color=ORANGE)
        ax.annotate(r"$R_{PZ}/\varepsilon = R_{PZ}/\cos\theta$", c_prime,
                    textcoords="offset points", xytext=(4, 8), fontsize=9, color=BLUE)
        ax.annotate("gain", (c + c_prime) / 2.0, textcoords="offset points", xytext=(10, -4),
                    fontsize=8, color=BLUE)
    else:  # park the read-out on the far side of the circle it labels, clear of the arcs
        ax.annotate(f"achieved {achieved:.0f} m",
                    achieved * 1.12 * np.array([math.cos(2.36), math.sin(2.36)]),
                    textcoords="offset points", xytext=(-2, 6), fontsize=8, color=RED,
                    ha="center")

    # equal spans, so "equal" aspect fills the square box; wide enough to hold the largest
    # achieved-miss circle any panel draws
    ax.set_xlim(-95.0, 115.0)
    ax.set_ylim(-95.0, 115.0)
    ax.set_aspect("equal")
    ax.set_box_aspect(1)
    ax.set_xticks([])
    ax.set_yticks([])
    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_visible(False)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path,
                   default=HERE.parent / "vault/derivations/img/robust-mvp-geometry.png")
    cfg = p.parse_args()
    cfg.out.parent.mkdir(parents=True, exist_ok=True)

    alpha = math.asin(D_MISS / RANGE)
    gamma = math.asin(RPZ / RANGE)
    theta_mvp = gamma - alpha

    sigma_phi = SIGMA_V / V_REL
    sigma_los = SIGMA_R / (RANGE * math.cos(gamma))

    # sigma_m has two terms and they behave completely differently under the manoeuvre, so split
    # them: position alone carries no theta (a constant angular buffer), velocity alone is all
    # theta (a buffer that shrinks as you turn).
    cases = (
        ("position only", 0.0, sigma_los),
        ("velocity only", sigma_phi, 0.0),
        ("both", sigma_phi, sigma_los),
    )

    fig, axes = plt.subplots(2, 2, figsize=(10.4, 10.0))
    draw(axes[0][0], alpha, gamma, theta_mvp, rmvp=False)
    axes[0][0].set_title(r"(a) MVP — rotate onto the tangent, $\theta=\gamma-\alpha$", fontsize=10)

    print(f"alpha  = {math.degrees(alpha):5.2f} deg   (|d_CPA| = {D_MISS:g} m, "
          f"range {RANGE:g} m)")
    print(f"gamma  = {math.degrees(gamma):5.2f} deg   (R_PZ = {RPZ:g} m)")
    print(f"theta  = {math.degrees(theta_mvp):5.2f} deg   -> achieved miss "
          f"{RANGE * math.sin(alpha + theta_mvp):5.1f} m = R_PZ")
    print(f"eps    = cos(theta) = {math.cos(theta_mvp):.4f};  R_PZ/eps = "
          f"{RPZ / math.cos(theta_mvp):.2f} m against R_PZ = {RPZ:g} m")
    print(f"\nsigma_phi = {sigma_phi:.4f} rad   sigma_los = {sigma_los:.4f} rad\n")
    print(f"{'case':>14} {'k*sigma_m':>10} {'theta*':>9} {'one-shot':>9} {'achieved':>9}")

    for ax, (label, s_phi, s_los) in zip(axes.ravel()[1:], cases, strict=True):
        th = rotation(alpha, gamma, K, s_phi, s_los)
        shot = min(gamma - alpha + K * angular_sigma(0.0, s_phi, s_los), 0.5 * math.pi - alpha)
        draw(ax, alpha, gamma, th, rmvp=True, one_shot=shot)
        ax.set_title(f"({'bcd'[cases.index((label, s_phi, s_los))]}) RMVP — {label}", fontsize=10)
        print(f"{label:>14} {math.degrees(K * angular_sigma(th, s_phi, s_los)):9.2f}d "
              f"{math.degrees(th):8.2f}d {math.degrees(shot):8.2f}d "
              f"{RANGE * math.sin(alpha + th):8.1f}m")

    fig.tight_layout()
    fig.savefig(cfg.out, dpi=130)
    print(f"\nwrote {cfg.out}")


if __name__ == "__main__":
    main()
