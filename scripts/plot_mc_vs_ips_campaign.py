"""Figures for the campaign of ``scripts/mc_vs_ips_campaign.py`` — four simple plots from its JSON.

Reads a ``campaign.json`` written by that runner and draws the four claims of
``vault/observations/mc-vs-ips-campaign.md``:

  1. ``mc-vs-ips-agreement.png`` -- the two estimates with their 95% intervals, and the ratio
     between them against the +-10% Monte Carlo precision the campaign was sized for.
  2. ``mc-vs-ips-gain.png``      -- MC time / IPS time against P(LoS), raw and at an equal interval
     width. The campaign fixed MC's precision at 100 events but let IPS's precision fall out of 20
     replications, so the raw ratio is not a ratio for equal answers; the corrected line is.
  3. ``mc-vs-ips-survival.png``  -- survival per shell, against the 0.5 the ladder aims for; the
     rungs at 0.25 are the ones ``build_ladder``'s ``step`` guard cost.
  4. ``mc-vs-ips-spread.png``    -- mean / geometric mean per cell: the degeneracy tell that the
     collapse count (0 in every cell here) does not give.
  5. ``mc-vs-ips-time.png``      -- wall time per scenario beside the P(LoS) that time bought. A
     shorter bar is a win only where the interval beside it is no wider; figure 2 divides that out.

    PYTHONPATH=. python scripts/plot_mc_vs_ips_campaign.py
    PYTHONPATH=. python scripts/plot_mc_vs_ips_campaign.py --json campaign.json --out-dir /tmp
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

MC_C, IPS_C = "tab:blue", "tab:red"
PART_C = {1: "tab:blue", 2: "tab:orange"}       # ring, traffic
PART_M = {1: "o", 2: "s"}


def load(path: Path) -> list[dict[str, Any]]:
    cells = json.loads(path.read_text())["cells"]
    return [c for c in cells if "mc" in c and "ips" in c]


def _asym(p: float, ci: list[float]) -> tuple[float, float]:
    """Error-bar half-widths for a point that need not sit at the centre of its interval."""
    return max(p - ci[0], 0.0), max(ci[1] - p, 0.0)


def plot_agreement(cells, out: Path) -> None:
    """Left: both estimates with their intervals. Right: IPS / MC against the MC precision."""
    labels = [c["label"] for c in cells]
    x = np.arange(len(cells), dtype=float)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12.5, 5.2))

    for dx, key, colour, name in ((-0.13, "mc", MC_C, "Monte Carlo"),
                                  (+0.13, "ips", IPS_C, "IPS")):
        p = np.array([c[key]["p"] for c in cells])
        err = np.array([_asym(c[key]["p"], c[key]["ci"]) for c in cells]).T
        a1.errorbar(x + dx, p, yerr=err, fmt=".", ms=11, capsize=4, lw=1.6,
                    color=colour, label=name)
    a1.set_yscale("log")
    a1.set_xticks(x, labels, rotation=20)
    a1.set_ylabel("P(LoS)")
    a1.set_title("Estimate and 95% interval — the two agree in all six cells")
    a1.grid(alpha=0.3, which="both")
    a1.legend()

    ratio = np.array([c["ips"]["p"] / c["mc"]["p"] for c in cells])
    a2.axhspan(0.9, 1.1, color="grey", alpha=0.18,
               label="MC precision at 100 events (±10%)")
    a2.axhline(1.0, color="k", lw=1.0)
    a2.bar(x, ratio - 1.0, bottom=1.0, width=0.5,
           color=[PART_C[c["part"]] for c in cells])
    for xi, r in zip(x, ratio, strict=True):
        a2.annotate(f"{r:.2f}", (xi, r), textcoords="offset points",
                    xytext=(0, 6 if r >= 1 else -14), ha="center", fontsize=9)
    a2.set_xticks(x, labels, rotation=20)
    a2.set_ylim(0.6, 1.4)
    a2.set_ylabel("IPS / MC")
    a2.set_title("Ratio of the two point estimates")
    a2.grid(alpha=0.3, axis="y")
    a2.legend(loc="lower right")

    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")


EQ_TEXT = (
    r"$\mathrm{gain}_{\mathrm{raw}}=\dfrac{T_{MC}}{T_{IPS}}$" "\n\n"
    r"$\mathrm{gain}_{\mathrm{equal}}=\dfrac{T_{MC}}{T_{IPS}}"
    r"\cdot\left(\dfrac{W_{MC}}{W_{IPS}}\right)^{\!2}$" "\n\n"
    r"$W=\ln\left(\dfrac{hi}{lo}\right)$ of the 95% interval"
)


def plot_gain(cells, out: Path) -> None:
    """Gain against rarity, as measured and after correcting the two methods to an equal interval.

    The solid line is the measured wall ratio. The faint line divides out the difference in
    interval width, which is the comparison that holds the answer quality constant; both intervals
    narrow as ``1/sqrt(effort)``, so the correction is the square of the ratio of the widths.
    """
    fig, ax = plt.subplots(figsize=(8.6, 5.6))
    ax.axhline(1.0, color="k", lw=1.0)
    ax.axhspan(0.1, 1.0, color="grey", alpha=0.12)
    for part, name in ((1, "ring (fixed geometry)"), (2, "traffic (sampled geometry)")):
        sel = [c for c in cells if c["part"] == part]
        p = [c["mc"]["p"] for c in sel]
        raw = [c["mc"]["wall_s"] / c["ips"]["wall_s"] for c in sel]
        eq = [r * (_log_width(c["mc"]["ci"]) / _log_width(c["ips"]["ci"])) ** 2
              for c, r in zip(sel, raw, strict=True)]
        ax.plot(p, raw, PART_M[part] + "-", color=PART_C[part], ms=9, lw=2.0,
                label=f"{name} — raw wall ratio")
        ax.plot(p, eq, PART_M[part] + ":", color=PART_C[part], ms=6, lw=1.2, alpha=0.45,
                label=f"{name} — at an equal interval")
        for c, pi, gi in zip(sel, p, raw, strict=True):
            ax.annotate(f"N={c['n']}", (pi, gi), textcoords="offset points",
                        xytext=(7, 5), fontsize=9)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.invert_xaxis()                      # rarer to the right
    ax.set_xlabel("P(LoS)   (rarer →)")
    ax.set_ylabel("gain = MC time / IPS time")
    ax.set_title("Raw wall ratio (solid) against the equal-interval gain (dotted)\n"
                 "the correction moves every ring cell below 1 (grey: MC is cheaper)")
    ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=8, loc="upper left")
    ax.text(0.03, 0.04, EQ_TEXT, transform=ax.transAxes, fontsize=9.5,
            va="bottom", ha="left",
            bbox=dict(boxstyle="round,pad=0.5", fc="white", ec="0.7", alpha=0.92))
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")
    for c in cells:
        raw = c["mc"]["wall_s"] / c["ips"]["wall_s"]
        w = _log_width(c["ips"]["ci"]) / _log_width(c["mc"]["ci"])
        print(f"    {c['label']:>13}  raw {raw:4.2f}x   width ratio {w:4.2f}   "
              f"equal-interval {raw / w**2:4.2f}x")


def plot_survival(cells, out: Path) -> None:
    """Survival per shell. The ladder aims for 0.5; a shell the step guard dropped gives 0.25."""
    fig, ax = plt.subplots(figsize=(9.0, 5.4))
    ax.axhline(0.5, color="k", lw=1.2, label="0.5 — the ladder target")
    ax.axhline(0.25, color="crimson", lw=1.2, ls="--",
               label="0.25 — one percentile shell dropped")
    for c in cells:
        s = c["ips"]["survival"]
        k = np.arange(1, len(s) + 1)
        ax.plot(k, s, PART_M[c["part"]] + "-", ms=5, lw=1.3,
                color=PART_C[c["part"]],
                alpha=0.45 + 0.2 * ((c["n"] - 2) % 3), label=c["label"])
        ax.annotate("last", (k[-1], s[-1]), textcoords="offset points",
                    xytext=(4, 2), fontsize=8, color="grey")
    ax.set_ylim(0.0, 1.0)
    ax.set_xlabel("shell number (start → rpz)")
    ax.set_ylabel("mean survival over 20 replications")
    ax.set_title("Survival per shell — the 0.25 rungs are the expensive ones,\n"
                 "the last rung is almost free and thus is waste")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")


def plot_spread(cells, out: Path) -> None:
    """Mean / geometric mean: how far the replications spread, when no replication collapsed."""
    labels = [c["label"] for c in cells]
    x = np.arange(len(cells), dtype=float)
    ratio = [c["ips"]["p"] / math.sqrt(c["ips"]["ci"][0] * c["ips"]["ci"][1]) for c in cells]
    fig, ax = plt.subplots(figsize=(7.6, 5.0))
    ax.bar(x, ratio, width=0.55, color=[PART_C[c["part"]] for c in cells])
    ax.axhline(1.3, color="crimson", lw=1.2, ls="--", label="1.3 — add particles above this")
    ax.axhline(1.0, color="k", lw=1.0)
    for xi, r, c in zip(x, ratio, cells, strict=True):
        ax.annotate(f"{r:.2f}\n{c['ips']['particles']} particles\n"
                    f"{c['ips']['collapsed']}/{c['ips']['reps']} collapsed",
                    (xi, r), textcoords="offset points", xytext=(0, 5),
                    ha="center", fontsize=8)
    ax.set_xticks(x, labels, rotation=20)
    ax.set_ylim(0.95, 2.2)
    ax.set_ylabel("mean / geometric mean of the 20 replications")
    ax.set_title("Spread between replications — ring N=2 is degenerate,\n"
                 "and its collapse count does not say so")
    ax.grid(alpha=0.3, axis="y")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")


def _log_width(ci: list[float]) -> float:
    """Interval width for a positive quantity: ``ln(hi/lo)``, which is scale free.

    Both intervals narrow as ``1/sqrt(effort)`` — MC with the event count, IPS with the replication
    count — so the effort to match a width scales as the square of the ratio of the widths.
    """
    return math.log(ci[1] / ci[0])


def plot_time(cells, out: Path) -> None:
    """Wall time per scenario, and the estimate that time bought.

    Read the two panels together: a shorter bar in (a) is only a win if the interval beside it in
    (b) is no wider. ``plot_gain`` divides the two out.
    """
    labels = [c["label"] for c in cells]
    x = np.arange(len(cells), dtype=float)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13.0, 5.4))

    # --- (a) wall time as the campaign ran it ---------------------------------------------------
    mc_t = np.array([c["mc"]["wall_s"] for c in cells])
    ips_t = np.array([c["ips"]["wall_s"] for c in cells])
    a1.bar(x - 0.19, mc_t, width=0.36, color=MC_C, label="Monte Carlo")
    a1.bar(x + 0.19, ips_t, width=0.36, color=IPS_C, label="IPS")
    for xi, m, i in zip(x, mc_t, ips_t, strict=True):
        a1.annotate(f"{m:.0f} s", (xi - 0.19, m), textcoords="offset points",
                    xytext=(0, 3), ha="center", fontsize=8)
        a1.annotate(f"{i:.0f} s", (xi + 0.19, i), textcoords="offset points",
                    xytext=(0, 3), ha="center", fontsize=8)
        a1.annotate(f"{m / i:.1f}×", (xi, max(m, i)), textcoords="offset points",
                    xytext=(0, 15), ha="center", fontsize=9, color="dimgrey")
    a1.set_yscale("log")
    a1.set_ylim(60, 8000)
    a1.set_xticks(x, labels, rotation=20)
    a1.set_ylabel("wall time on 100 workers [s]")
    a1.set_title("(a) Time the campaign spent\n"
                 f"total {mc_t.sum():.0f} s MC + {ips_t.sum():.0f} s IPS")
    a1.grid(alpha=0.3, axis="y", which="both")
    a1.legend()

    # --- (b) what that time bought --------------------------------------------------------------
    for dx, key, colour, name in ((-0.13, "mc", MC_C, "Monte Carlo"),
                                  (+0.13, "ips", IPS_C, "IPS")):
        p = np.array([c[key]["p"] for c in cells])
        err = np.array([_asym(c[key]["p"], c[key]["ci"]) for c in cells]).T
        a2.errorbar(x + dx, p, yerr=err, fmt=".", ms=11, capsize=4, lw=1.6,
                    color=colour, label=name)
    a2.set_yscale("log")
    a2.set_xticks(x, labels, rotation=20)
    a2.set_ylabel("P(LoS)")
    a2.set_title("(b) The estimate it bought\nall six agree; the ring N=2 IPS interval is wide")
    a2.grid(alpha=0.3, which="both")
    a2.legend()

    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", type=Path, default=Path("campaign.json"))
    ap.add_argument("--out-dir", type=Path, default=Path("vault/observations/img"))
    a = ap.parse_args()

    cells = load(a.json)
    a.out_dir.mkdir(parents=True, exist_ok=True)
    plot_agreement(cells, a.out_dir / "mc-vs-ips-agreement.png")
    plot_gain(cells, a.out_dir / "mc-vs-ips-gain.png")
    plot_survival(cells, a.out_dir / "mc-vs-ips-survival.png")
    plot_spread(cells, a.out_dir / "mc-vs-ips-spread.png")
    plot_time(cells, a.out_dir / "mc-vs-ips-time.png")


if __name__ == "__main__":
    main()
