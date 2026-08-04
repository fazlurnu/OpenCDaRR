"""The three pictures the RMVP derivation needs — the fixed point, the target, and the SNR.

Written for ``vault/derivations/robust-mvp-resolution.md``, which is where these are explained.
Each panel answers one question that the algebra alone leaves open:

1. **Why is the rule implicit?** The angular margin the confidence demands *falls* as the
   manoeuvre grows, so the requirement and the means of meeting it cross at one point rather than
   the requirement being a number you can look up.
2. **Is this just an adaptive margin?** Any rotation rule can be written as MVP aiming at an
   enlarged target. The panel shows what target this one ends up at, against the range — the miss
   distance no manoeuvre can exceed, and the bound an offset-domain margin has no way to respect.
3. **What does the manoeuvre actually buy?** The angular noise on the relative-velocity direction,
   before and after, against the position term that no velocity change can touch.

    python robust-mvp/fig_mechanism.py
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import NamedTuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from rmvp import angular_sigma, relative_sigmas, rotation  # noqa: E402

from opencdarr.cr.mvp import _BIAS_EPS  # noqa: E402
from opencdarr.relative import relative_enu  # noqa: E402
from opencdarr.scenario import create_conflict  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

SPEED, RPZ, MARGIN, TLOS, DCPA = 10.2889, 50.0, 1.05, 180.0, 0.0
POS_CI95, VEL_CI95 = 10.0, 3.0
K = 1.6448536269514722  # Phi^-1(0.95)
GREY, RED, BLUE = "0.45", "#d62728", "#1f77b4"
FOCUS = 2.0  # deg: the crossing angle panel (a) works through


class Geometry(NamedTuple):
    """Everything the rules read off one crossing angle."""

    alpha: float  # rad: v_rel's angular offset from the line of sight, on MVP's _BIAS_EPS floor
    gamma: float  # rad: the offset that puts the miss distance at margin * rpz
    dist: float  # m: range
    v_rel: float  # m/s
    sigma_r: float  # m: relative position, per axis
    sigma_v: float  # m/s: relative velocity, per axis

    @property
    def sigma_phi(self) -> float:
        """Angular noise on the relative-velocity direction, before any manoeuvre [rad]."""
        return self.sigma_v / self.v_rel

    @property
    def sigma_los(self) -> float:
        """The position side of the angular margin [rad] — untouchable by a velocity change."""
        return self.sigma_r / (self.dist * math.cos(self.gamma))

    @property
    def t_cpa(self) -> float:
        """Time to closest approach [s] — what an offset-domain margin scales sigma_v by."""
        return self.dist * math.cos(self.alpha) / self.v_rel

    @property
    def theta(self) -> float:
        """RMVP's self-consistent rotation [rad]."""
        return rotation(self.alpha, self.gamma, K, self.sigma_phi, self.sigma_los)


def geometry(dpsi: float) -> Geometry:
    """The encounter at one crossing angle, at the campaign's declared accuracy.

    ``alpha`` uses MVP's own ``_BIAS_EPS`` floor on the miss distance, because at ``dcpa`` = 0 that
    floor is what both resolvers actually see.
    """
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=SPEED,
                        pos_ci95=POS_CI95, vel_ci95=VEL_CI95)
    intr = create_conflict(own, intr_id="INT", dpsi=dpsi, dcpa=DCPA, tlos=TLOS, rpz=RPZ, side=1)
    rel = relative_enu(own, intr)
    sigma_r, sigma_v = relative_sigmas(own, intr)
    return Geometry(
        alpha=math.asin(_BIAS_EPS / rel.dist),
        gamma=math.asin(min(RPZ * MARGIN / rel.dist, 1.0)),
        dist=rel.dist,
        v_rel=math.hypot(rel.vx, rel.vy),
        sigma_r=sigma_r,
        sigma_v=sigma_v,
    )


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path,
                   default=HERE.parent / "vault/derivations/img/robust-mvp-mechanism.png")
    cfg = p.parse_args()
    cfg.out.parent.mkdir(parents=True, exist_ok=True)

    fig, (ax_a, ax_b, ax_c) = plt.subplots(1, 3, figsize=(13.0, 4.4))

    # (a) the fixed point ----------------------------------------------------------------------
    g0 = geometry(FOCUS)
    alpha, gamma, sigma_phi, sigma_los = g0.alpha, g0.gamma, g0.sigma_phi, g0.sigma_los
    theta = np.linspace(0.0, 0.5 * math.pi - alpha, 600)
    required = gamma + K * np.array([angular_sigma(t, sigma_phi, sigma_los) for t in theta])
    supplied = alpha + theta
    star = g0.theta

    ax_a.plot(theta, required, color=RED, lw=1.8,
              label=r"required  $\gamma + k\,\sigma_m(\theta)$")
    ax_a.plot(theta, supplied, color=BLUE, lw=1.8, label=r"supplied  $\alpha + \theta$")
    ax_a.axhline(gamma, color=GREY, ls="--", lw=1.0)
    ax_a.plot([gamma - alpha], [gamma], marker="o", ms=6, color=GREY, zorder=5)
    ax_a.plot([star], [alpha + star], marker="*", ms=14, color=RED, zorder=5)
    ax_a.annotate(r"MVP  ($k=0$)", (gamma - alpha, gamma), textcoords="offset points",
                  xytext=(6, -14), fontsize=8, color=GREY)
    ax_a.annotate(r"RMVP  $\theta^*$", (star, alpha + star), textcoords="offset points",
                  xytext=(-52, 10), fontsize=8, color=RED)
    ax_a.set_yscale("log")
    ax_a.set_ylim(0.02, 20.0)
    ax_a.set_xlabel(r"rotation $\theta$ [rad]")
    ax_a.set_ylabel("angular margin [rad]")
    ax_a.legend(frameon=False, fontsize=8, loc="upper right")
    ax_a.set_box_aspect(1)
    print(f"(a) dpsi={FOCUS:g}deg: gamma={gamma:.4f}, sigma_phi={sigma_phi:.3f}, "
          f"sigma_los={sigma_los:.4f} rad; one-shot demand k*sigma_m(0)+gamma-alpha="
          f"{gamma + K * sigma_phi - alpha:.3f} rad, self-consistent theta*={star:.4f} rad")

    # (b) the target each rule aims at ----------------------------------------------------------
    angles = np.geomspace(1.0, 90.0, 120)
    rows = [geometry(float(d)) for d in angles]
    dists = np.array([g.dist for g in rows])
    mvp = np.full_like(dists, RPZ * MARGIN)
    # RMVP inflates the *angle*, so its equivalent miss-distance target is what its rotation
    # actually aims the offset at, |r| sin(alpha + theta*).
    rmvp = np.array([g.dist * math.sin(min(g.alpha + g.theta, 0.5 * math.pi)) for g in rows])

    ax_b.plot(angles, dists, color="0.1", ls="--", lw=1.2, label=r"range $|\mathbf{r}|$ (bound)")
    ax_b.plot(angles, rmvp, color=RED, lw=1.8, label="RMVP target")
    ax_b.plot(angles, mvp, color=GREY, lw=1.8, label="MVP target")
    ax_b.set_xscale("log")
    ax_b.set_yscale("log")
    ax_b.set_xlabel(r"crossing angle $\Delta\psi$ [deg]")
    ax_b.set_ylabel("targeted miss distance [m]")
    ax_b.legend(frameon=False, fontsize=8, loc="upper left")
    ax_b.set_box_aspect(1)
    # The offset-domain alternative is not plotted — this study compares MVP against RMVP — but it
    # is still reported, because the derivation quotes it as the reason not to write the constraint
    # on the distance: R_eff = margin*rpz + k*sqrt(sigma_r^2 + t_cpa^2 sigma_v^2) is unbounded and
    # asks for miss distances the range cannot supply.
    offset_domain = np.array([RPZ * MARGIN + K * math.hypot(g.sigma_r, g.t_cpa * g.sigma_v)
                              for g in rows])
    over = angles[offset_domain > dists]
    print(f"(b) RMVP stays under the range everywhere (max ratio "
          f"{np.max(rmvp / dists):.3f}x). Not plotted, for reference: the "
          f"offset-domain constraint asks for more than the range itself below "
          f"{over.max():.1f} deg (max ratio {np.max(offset_domain / dists):.1f}x)")

    # (c) what the manoeuvre buys ---------------------------------------------------------------
    before = np.array([g.sigma_phi for g in rows])
    after = np.array([g.sigma_phi * math.cos(g.theta) for g in rows])
    los = np.array([g.sigma_los for g in rows])
    ax_c.plot(angles, before, color=GREY, lw=1.8, label=r"$\sigma_v/|\mathbf{v}|$  before")
    ax_c.plot(angles, after, color=RED, lw=1.8, label=r"$\sigma_v/|\mathbf{v}'|$  after")
    ax_c.plot(angles, los, color=BLUE, lw=1.8, label=r"$\sigma_r/(|\mathbf{r}|\cos\gamma)$")
    ax_c.set_xscale("log")
    ax_c.set_yscale("log")
    ax_c.set_xlabel(r"crossing angle $\Delta\psi$ [deg]")
    ax_c.set_ylabel("angular noise [rad]")
    ax_c.legend(frameon=False, fontsize=8, loc="upper right")
    ax_c.set_box_aspect(1)
    print(f"(c) the manoeuvre divides the velocity term by {before[0] / after[0]:.1f}x at "
          f"{angles[0]:.0f} deg and {before[-1] / after[-1]:.2f}x at {angles[-1]:.0f} deg; "
          f"the position term is untouched at both")

    fig.tight_layout()
    fig.savefig(cfg.out, dpi=130)
    print(f"\nwrote {cfg.out}")


if __name__ == "__main__":
    main()
