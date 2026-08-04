"""The campaign's two figures — what the resolver buys, and what it costs.

Reads ``ips_results.json`` (the rare-event estimate) and ``probe_results.json`` (the manoeuvring
cost, which plain MC measures perfectly well even though it cannot measure the rare event). Writes
``fig_ips.png``.

    python robust-mvp/plots.py
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from conditions_rm import DPSI, RESOLVERS  # noqa: E402

COLOURS = {"MVP": "0.45", "RMVP0.95": "#d62728"}
WIDTH = 0.36


def _index(rows: list[dict], key: str) -> dict[tuple[float, float, str], dict]:
    """Campaign rows indexed by (dpsi, vel_ci95, resolver)."""
    return {(r["dpsi"], r["vel_ci95"], r["resolver"]): r for r in rows} if key else {}


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--ips", type=Path, default=HERE / "ips_results.json")
    p.add_argument("--probe", type=Path, default=HERE / "probe_results.json")
    p.add_argument("--out", type=Path, default=HERE / "fig_ips.png")
    cfg = p.parse_args()

    ips = _index(json.loads(cfg.ips.read_text()), "ips")
    probe = {(r["dpsi"], r["vel_ci95"], r["resolver"]): r
             for r in json.loads(cfg.probe.read_text()).values()}
    noise = sorted({v for _, v, _ in ips})
    names = [n for n in RESOLVERS if any(n == r for _, _, r in ips)]

    fig, axes = plt.subplots(1, 2, figsize=(10.0, 4.6))
    x = np.arange(len(DPSI), dtype=float)

    ax = axes[0]
    for i, name in enumerate(names):
        offset = (i - (len(names) - 1) / 2) * WIDTH
        probs, lo, hi, collapsed = [], [], [], []
        for dpsi in DPSI:
            row = ips.get((dpsi, noise[-1], name))
            probs.append(row["prob"] if row else np.nan)
            lo.append(row["prob"] - row["ci_lo"] if row else 0.0)
            hi.append(row["ci_hi"] - row["prob"] if row else 0.0)
            collapsed.append(row["n_collapsed"] if row else 0)
        ax.bar(x + offset, probs, WIDTH, color=COLOURS.get(name, "0.7"), label=name)
        ax.errorbar(x + offset, probs, yerr=[np.abs(lo), np.abs(hi)], fmt="none",
                    ecolor="0.15", elinewidth=1.0, capsize=3)
        for xi, prob, n_bad in zip(x + offset, probs, collapsed, strict=True):
            if n_bad:  # a collapsed ladder is an upper bound, not an estimate
                ax.text(xi, prob, f"{n_bad}/{ips[(DPSI[0], noise[-1], name)]['reps']}",
                        ha="center", va="bottom", fontsize=7)
    ax.set_yscale("log")
    ax.set_xticks(x, [f"{d:g}" for d in DPSI])
    ax.set_xlabel("crossing angle [deg]")
    ax.set_ylabel("P(LoS)")
    ax.legend(frameon=False, fontsize=9)
    ax.set_box_aspect(1)

    ax = axes[1]
    for i, name in enumerate(names):
        offset = (i - (len(names) - 1) / 2) * WIDTH
        dev = [probe[(d, noise[-1], name)]["dev_rate"] for d in DPSI]
        ax.bar(x + offset, dev, WIDTH, color=COLOURS.get(name, "0.7"), label=name)
    ax.set_xticks(x, [f"{d:g}" for d in DPSI])
    ax.set_xlabel("crossing angle [deg]")
    ax.set_ylabel("deviation from nominal [m/s]")
    ax.set_box_aspect(1)

    fig.tight_layout()
    fig.savefig(cfg.out, dpi=130)
    print(f"wrote {cfg.out}")


if __name__ == "__main__":
    main()
