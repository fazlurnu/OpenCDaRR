"""The IPS campaign — P(LoS) per cell by multilevel splitting on the running-minimum separation.

Plain Monte Carlo cannot do this job here: 3200 probe encounters recorded **zero** losses of
separation, so every cell would read 0.000 with the same interval, a number set by the sample size
rather than by the resolver. IPS (:mod:`opencdarr.ips`, ADR 0017) concentrates effort on the
trajectories heading toward the rare set and returns the small probability with a usable interval.

The shell ladder is built **per cell** from that cell's own probe distribution
(:func:`~conditions_rm.ladder`, ``probe_mc.py`` → ``probe_results.json``). One hand-tuned sequence
cannot serve both crossing angles: at Δψ = 2° the pair starts 114.6 m apart and at 30° it starts
1008.7 m apart.

Run ``--pilot`` first: it prints the per-level survival fractions, which is the only way to tell a
ladder spaced too aggressively (a level with no survivors reports ``P = 0``, which is not a real
zero) from one that is merely rare.

    python robust-mvp/ips_run.py --pilot                  # survival diagnostics, cheap
    python robust-mvp/ips_run.py --n 400 --reps 4         # production
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))  # runnable from any cwd

from conditions_rm import RESOLVERS, cells, ladder  # noqa: E402

from opencdarr.parallel import estimate_rare_prob  # noqa: E402


def ladders(path: Path, tail_levels: int, survival: float) -> dict[str, tuple[float, ...]]:
    """The per-cell shell sequences, rebuilt from the probe's recorded minimum separations.

    Rebuilt rather than read back, so the ladder can be retuned — the one knob a collapsing level
    calls for — without paying for the probe again.
    """
    if not path.exists():
        raise SystemExit(f"{path} not found — run `python robust-mvp/probe_mc.py` first")
    probe = json.loads(path.read_text())
    return {
        key: ladder(row["min_sep"], tail_levels=tail_levels, survival=survival)
        for key, row in probe.items()
    }


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--pilot", action="store_true", help="small run, print per-level survival")
    p.add_argument("--n", type=int, default=400, help="particles per shell")
    p.add_argument("--reps", type=int, default=4, help="independent replications (the CI)")
    p.add_argument("--jobs", type=int, default=-1)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--probe", type=Path, default=HERE / "probe_results.json")
    p.add_argument("--tail-levels", type=int, default=12,
                   help="shells below the smallest observed run")
    p.add_argument("--survival", type=float, default=0.6,
                   help="target survival fraction per shell in the observed range")
    p.add_argument("--out", type=Path, default=HERE / "ips_results.json")
    p.add_argument("--resolvers", nargs="+", default=None, choices=tuple(RESOLVERS),
                   help="run only these resolvers (default: all)")
    p.add_argument("--vel-ci95", nargs="+", type=float, default=None,
                   help="run only cells declaring these velocity accuracies (default: all)")
    p.add_argument("--dpsi", nargs="+", type=float, default=None,
                   help="run only these crossing angles (default: all)")
    cfg = p.parse_args()
    n_particles = 200 if cfg.pilot else cfg.n
    reps = 2 if cfg.pilot else cfg.reps

    shells = ladders(cfg.probe, cfg.tail_levels, cfg.survival)
    grid = cells(
        tuple(cfg.resolvers) if cfg.resolvers else None,
        tuple(cfg.vel_ci95) if cfg.vel_ci95 else None,
        tuple(cfg.dpsi) if cfg.dpsi else None,
    )
    # Resumable: each cell is appended as it finishes and already-done cells are skipped, so a
    # multi-hour campaign survives being interrupted and can be extended a few cells at a time.
    rows: list[dict[str, object]] = (
        json.loads(cfg.out.read_text()) if cfg.out.exists() and not cfg.pilot else []
    )
    done = {r["key"] for r in rows}
    todo = [c for c in grid if cfg.pilot or c.key not in done]
    print(f"IPS — {len(todo)}/{len(grid)} cells to run, {n_particles} particles x {reps} reps, "
          f"per-cell ladders from {cfg.probe.name}", flush=True)
    t0 = time.time()
    for cell in todo:
        t_cell = time.time()
        levels = shells[cell.key]
        start = cell.build_particle()
        est = estimate_rare_prob(
            lambda _seq, _s=start: _s, levels,
            n_particles=n_particles, reps=reps, seed=cfg.seed, n_jobs=cfg.jobs,
        )
        rows = [r for r in rows if r["key"] != cell.key]
        rows.append({
            "key": cell.key, "label": cell.label,
            "dpsi": cell.dpsi, "pos_ci95": cell.pos_ci95, "vel_ci95": cell.vel_ci95,
            "resolver": cell.resolver, "prob": est.prob, "ci_lo": est.ci[0], "ci_hi": est.ci[1],
            "n_collapsed": est.n_collapsed, "n_particles": n_particles, "reps": reps,
            "shells": list(levels), "seed": cfg.seed,
            # per-replication estimates and survival, so a later run on a different seed can be
            # pooled into these rather than replacing them, and a marginal shell stays visible
            "rep_probs": [r.prob for r in est.reps],
            "rep_survival": [list(r.survival) for r in est.reps],
        })
        print(f"  {cell.label:44s} P(LoS)={est.prob:9.3e}  "
              f"CI [{est.ci[0]:8.2e}, {est.ci[1]:8.2e}]  collapsed {est.n_collapsed}/{reps}"
              f"  ({time.time() - t_cell:.0f}s)", flush=True)
        if not cfg.pilot:
            cfg.out.write_text(json.dumps(rows, indent=1))  # checkpoint after every cell
        else:
            good = [r for r in est.reps if r.collapsed_at is None] or list(est.reps)
            surv = [float(np.mean([r.survival[k] for r in good if k < len(r.survival)]))
                    for k in range(len(levels))]
            print("      shell:  " + "  ".join(f"{d:5.1f}" for d in levels), flush=True)
            print("      surv :  " + "  ".join(f"{s:5.2f}" for s in surv), flush=True)

    if not cfg.pilot and todo:
        print(f"\nwrote {cfg.out} ({len(rows)} cells)", flush=True)
    print(f"({time.time() - t0:.0f} s)", flush=True)


if __name__ == "__main__":
    main()
