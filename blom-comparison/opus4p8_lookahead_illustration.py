"""Illustrate the look-ahead conflict coordinate (Blom eq. 10.7, horizontal) — two panels.

(left)  the relative geometry: predicted straight-line path r + tau*v, its closest approach (dcpa).
(right) the nested conflict levels as nested BOXES in the (look-ahead tau, predicted separation s)
        plane: level D_k is the box [0, tau_k] x [0, d_k]; the pair is "in D_k" exactly when the
        predicted-separation curve s(tau) dips into that box; phi = deepest box reached / m.

Writes to vault/observations/img/lookahead-coordinate.png. See
vault/observations/lookahead-conflict-coordinate.md.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402

# one example relative geometry (own at origin), chosen to give a mid-deep phi
r = np.array([447.8, 232.1])   # relative position [m]  (pair ~500 m apart)
v = np.array([-23.5, -8.55])   # relative velocity [m/s] (closing; |v| = 25 m/s)

RPZ, D_MAX, TAU_MAX, M = 50.0, 95.0, 55.0, 14
k = np.arange(1, M + 1)
d_k = D_MAX - (D_MAX - RPZ) * k / M      # miss threshold per level (shrinks to rpz)
tau_k = TAU_MAX * (1 - k / M)            # look-ahead per level (shrinks to 0)


def sep(t: np.ndarray) -> np.ndarray:
    """Predicted separation |r + t*v| at look-ahead t."""
    px = r[0] + v[0] * t
    py = r[1] + v[1] * t
    return np.hypot(px, py)


t_star = -float(np.dot(r, v)) / float(np.dot(v, v))        # time of closest approach
dcpa = float(sep(np.array([max(t_star, 0.0)]))[0])
# smin_k = min over [0, tau_k] of sep ; deepest level reached = phi * M
smin = np.array([float(sep(np.array([min(max(t_star, 0.0), tk)]))[0]) for tk in tau_k])
in_level = smin <= d_k
deepest = int(k[in_level].max()) if in_level.any() else 0
phi = deepest / M

fig, (axA, axB) = plt.subplots(1, 2, figsize=(12.4, 5.4))

# ---- Panel A: relative geometry -------------------------------------------------------------------
tt = np.linspace(0, TAU_MAX, 100)
path = np.array([r[0] + v[0] * tt, r[1] + v[1] * tt])
axA.plot(path[0], path[1], "--", color="0.5", lw=1.4, label="predicted path  r + τ·v")
for tm in (10, 20, 30, 40, 50):                            # look-ahead ticks along the path
    p = r + v * tm
    axA.plot(*p, ".", color="0.5", ms=5)
    if tm in (20, 40):                                     # label only a couple, to avoid crowding
        axA.annotate(f"τ={tm}s", p, textcoords="offset points", xytext=(6, -12),
                     fontsize=8, color="0.5")
for rad, lab in ((RPZ, "rpz"), (D_MAX, "d₁≈")):            # a couple of miss rings around own
    axA.add_patch(Circle((0, 0), rad, fill=False, ls=":", ec="0.75", lw=1.0))
cpa = r + v * t_star
axA.plot([0, cpa[0]], [0, cpa[1]], ":", color="#d62728", lw=1.2)
axA.plot(*cpa, "*", color="#d62728", ms=13, label=f"closest approach  (dcpa={dcpa:.0f} m)")
axA.annotate("v", r + v * 6, textcoords="offset points", xytext=(6, -2), fontsize=11)
axA.annotate("", xy=r + v * 6, xytext=tuple(r),
             arrowprops=dict(arrowstyle="-|>", color="#1f77b4", lw=1.8))
axA.plot(0, 0, "o", color="#1f77b4", ms=9)
axA.annotate("own", (0, 0), textcoords="offset points", xytext=(6, 6), fontsize=9)
axA.plot(*r, "o", color="#ff7f0e", ms=9)
axA.annotate("intruder", r, textcoords="offset points", xytext=(6, 6), fontsize=9)
axA.set_aspect("equal")
axA.set_xlabel("East [m]")
axA.set_ylabel("North [m]")
axA.set_title("relative geometry: predicted path and closest approach", fontsize=10)
axA.legend(frameon=False, fontsize=8, loc="lower left")

# ---- Panel B: nested conflict levels as nested boxes ----------------------------------------------
cmap = plt.get_cmap("viridis")
for i in range(M):                                         # nested boxes, corner at origin
    col = cmap(i / (M - 1))
    filled = (i + 1) == deepest
    axB.add_patch(Rectangle((0, 0), tau_k[i], d_k[i], fill=filled, ec=col,
                            fc=(col if filled else "none"), alpha=(0.30 if filled else 1.0),
                            lw=(2.4 if filled else 1.0), zorder=(3 if filled else 1)))
axB.plot(tt, sep(tt), color="#d62728", lw=2.2, label="predicted separation  s(τ)=|r+τ·v|", zorder=5)
axB.plot(max(t_star, 0.0), dcpa, "*", color="#d62728", ms=13, zorder=6)
axB.annotate(f"dcpa = {dcpa:.0f} m", (max(t_star, 0.0), dcpa), textcoords="offset points",
             xytext=(8, 6), fontsize=9, color="#d62728")
axB.annotate(f"deepest box reached\n→ φ = {deepest}/{M} = {phi:.2f}",
             (tau_k[deepest - 1], d_k[deepest - 1]), textcoords="offset points", xytext=(10, -34),
             fontsize=9, arrowprops=dict(arrowstyle="->", color="0.3"))
axB.annotate("level D_k = box [0, τ_k] × [0, d_k]\nin D_k  ⇔  the curve dips into box k\n"
             "(k=1 outer/loose → k=14 inner = collision)",
             (0.03, 0.97), xycoords="axes fraction", ha="left", va="top", fontsize=8.5, color="0.25")
axB.set_xlim(0, TAU_MAX)
axB.set_ylim(0, 160)
axB.set_xlabel("look-ahead  τ  [s]   (horizon shrinks →)")
axB.set_ylabel("predicted separation  s  [m]")
axB.set_title("nested conflict levels → importance function φ", fontsize=10)
axB.legend(frameon=False, fontsize=8, loc="upper right")

fig.tight_layout()
out = Path(__file__).resolve().parents[1] / "vault/observations/img/lookahead-coordinate.png"
out.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(out, dpi=140)
print(f"phi = {deepest}/{M} = {phi:.3f}, dcpa = {dcpa:.1f} m, t_cpa = {t_star:.1f} s")
print(f"wrote {out}")
