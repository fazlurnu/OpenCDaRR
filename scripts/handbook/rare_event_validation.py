"""Handbook figures: the CNS-uncertainty validation sweep for rare-event simulation.

Data is not simulated here — it is transcribed from two real sweeps already run and logged under
``scripts/``:

  - ``cns_sweep_20260728_085447/`` — the original 13-cell sweep (Monte Carlo anchor + IPS at
    10 reps x 10 000 particles x 17 shells per cell). One 14th cell, pos 10 m / rx 0.12, errored
    and produced no estimate; it is not included here.
  - ``ips_rerun_20260729_072950/`` — IPS rerun on the same 13 cells, same seeds, to check
    reproducibility. Every P(IPS) matches the original run exactly; wall-clock time dropped
    substantially because of a new lockstep task schedule (200 tasks/level over 100 workers,
    vs. one process per replication before). Monte Carlo was not rerun (it does not depend on the
    IPS scheduler), so its timing is the original sweep's.

The rx = 0.12 cell (pos 3 m) is dropped from these figures: it is the one cell with a visibly
non-Gaussian, wide IPS interval, and including it stretches both figures' axes without adding to
the point being made here (does IPS track Monte Carlo, and how much does it cost).

Both figures compare only Monte Carlo against the **current** IPS (the rerun's lockstep schedule);
the original, slower IPS run is not shown. pos_ci95 = 3 m and 10 m are different navigation-noise
conditions, not repeats of the same one, so the probability figure keeps them as two subplots
rather than averaging them together.

    PYTHONPATH=. python scripts/handbook/rare_event_validation.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

IMG = Path.home() / "Projects/opencdarr.github.io/docs/assets/img"
BLUE, ORANGE, ORANGE_LIGHT = "#1f77b4", "#ff7f0e", "#ffbf86"

# pos [m], rx, P_mc, mc_lo, mc_hi, P_ips, ips_lo, ips_hi, t_mc [s], t_ips_old [s], t_ips_new [s]
# transcribed from the two sweep log directories described above (rx = 0.12 dropped).
CELLS = [
    (3, 1.00, 0.00009, 0.00008, 0.00010, 0.000092, 0.000057, 0.000114, 1582, 918, 244),
    (3, 0.99, 0.00009, 0.00007, 0.00010, 0.000082, 0.000064, 0.000095, 1593, 936, 246),
    (3, 0.90, 0.00005, 0.00004, 0.00006, 0.000056, 0.000039, 0.000067, 1596, 951, 265),
    (3, 0.70, 0.00003, 0.00003, 0.00004, 0.000048, 0.000009, 0.000044, 1605, 1000, 250),
    (3, 0.50, 0.00003, 0.00002, 0.00004, 0.000025, 0.000009, 0.000031, 1592, 1012, 275),
    (3, 0.30, 0.00004, 0.00003, 0.00005, 0.000034, 0.000007, 0.000032, 1641, 1066, 274),
    (10, 1.00, 0.00005, 0.00004, 0.00006, 0.000057, 0.000039, 0.000070, 1587, 953, 245),
    (10, 0.99, 0.00005, 0.00004, 0.00006, 0.000054, 0.000033, 0.000066, 1623, 968, 263),
    (10, 0.90, 0.00005, 0.00004, 0.00006, 0.000041, 0.000026, 0.000050, 1624, 963, 254),
    (10, 0.70, 0.00003, 0.00002, 0.00004, 0.000016, 0.000010, 0.000019, 1613, 994, 266),
    (10, 0.50, 0.00003, 0.00003, 0.00004, 0.000039, 0.000013, 0.000049, 1619, 1031, 271),
    (10, 0.30, 0.00003, 0.00003, 0.00004, 0.000023, 0.000008, 0.000029, 1616, 1083, 276),
]


def p_vs_rx_figure(out: Path) -> None:
    """P(LoS) against reception probability, Monte Carlo vs IPS (current schedule), one subplot per
    pos_ci95 (3 m, 10 m) — different navigation-noise conditions, so kept separate rather than
    averaged. Point estimates only; see validation.md for each estimator's 95% CI."""
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 5.4), sharey=True)
    for ax, pos in zip(axes, (3, 10), strict=True):
        rows = sorted((c for c in CELLS if c[0] == pos), key=lambda r: r[1])
        rx = [r[1] for r in rows]
        p_mc = [r[2] for r in rows]
        p_ips = [r[5] for r in rows]
        ax.plot(rx, p_mc, "o-", color=BLUE, ms=6, lw=1.5, label="Monte Carlo")
        ax.plot(rx, p_ips, "s-", color=ORANGE, ms=6, lw=1.5, label="IPS")
        ax.set_xlabel("reception probability rx")
        ax.set_title(f"pos_ci95 = {pos} m", fontsize=10)
        ax.set_box_aspect(1)
    axes[0].set_ylabel("P(LoS)")
    axes[0].legend(fontsize=8)

    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    print("wrote", out)


def timing_figure(out: Path) -> None:
    """Wall-clock cost per cell: the Monte Carlo anchor vs the IPS rerun (current lockstep
    schedule). Bars are the mean across the 12 cells; whiskers are the min-max range; individual
    cells are shown as jittered points."""
    t_mc = np.array([c[8] for c in CELLS]) / 60.0
    t_new = np.array([c[10] for c in CELLS]) / 60.0
    series = [("Monte Carlo\n(2M encounters)", t_mc, BLUE),
              ("IPS\n(10 reps x 10k particles)", t_new, ORANGE)]

    fig, ax = plt.subplots(figsize=(3.4, 3.8))
    rng = np.random.default_rng(0)
    for i, (label, t, col) in enumerate(series):
        mean = t.mean()
        ax.bar(i, mean, color=col, width=0.5,
               yerr=[[mean - t.min()], [t.max() - mean]], capsize=3, ecolor="0.3")
        jitter = rng.uniform(-0.1, 0.1, size=t.size)
        ax.scatter(np.full(t.size, i) + jitter, t, color="0.25", s=7, zorder=3, alpha=0.7)
        ax.text(i, mean + (t.max() - mean) + 1.0, f"{mean:.1f} min", ha="center", fontsize=7)

    ax.set_xticks(range(len(series)))
    ax.set_xticklabels([s[0] for s in series], fontsize=7)
    ax.set_ylabel("wall-clock time [min]", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.set_ylim(0, t_mc.max() * 1.1)
    ax.set_box_aspect(1)

    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    print("wrote", out)


def main() -> None:
    IMG.mkdir(parents=True, exist_ok=True)
    p_vs_rx_figure(IMG / "rare-event-p-vs-rx.png")
    timing_figure(IMG / "rare-event-timing.png")


if __name__ == "__main__":
    main()
