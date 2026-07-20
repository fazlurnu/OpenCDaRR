"""Fixed-crossing-angle IPR sweep, parallelised with joblib.

For each crossing angle, place one ``dcpa = 0`` conflict pair and run ``--n`` independent noise
realisations through the encounter loop; report IPR = 1 − LoS / n (denominator = n_pair, since
every pair is a constructed conflict). LoS is measured on the true separation.

Examples
--------
    python scripts/ipr_angle_sweep.py                          # MVP, default sweep
    python scripts/ipr_angle_sweep.py --resolver vo            # same, VO shortest-way-out
    python scripts/ipr_angle_sweep.py --ci95 10 --velo 1 --angles 2 10 45 90 --n 500 --jobs 4
"""

from __future__ import annotations

import argparse
import time
import numpy as np
from joblib import Parallel, delayed

from opencdarr.cd import StateBased
from opencdarr.cns import GpsNavigation
from opencdarr.cr import MVP, VO
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr import PastCPA
from opencdarr.loop import run_encounter
from opencdarr.performance import M600
from opencdarr.rng import generator, root_seed_sequence, spawn
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState

_CI95_TO_STD = 2.4477  # velocity CI95 (m/s) -> per-axis 1-sigma


def _resolver(name: str, margin: float) -> ConflictResolver:
    return {"mvp": MVP, "vo": VO}[name](margin=margin)


def _one(dpsi: float, seq, cfg: argparse.Namespace) -> int:
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=cfg.speed)
    intr = create_conflict(
        own, intr_id="INT", dpsi=dpsi, dcpa=0.0, tlos=cfg.tlos, rpz=cfg.rpz, side=1)
    nav = GpsNavigation(cfg.ci95, cfg.velo / _CI95_TO_STD)
    out = run_encounter(
        own, intr, perf=M600, rpz=cfg.rpz, t_lookahead=cfg.lookahead, dt=cfg.dt,
        detector=StateBased(), resolver=_resolver(cfg.resolver, cfg.margin),
        recovery=PastCPA(bouncing_guard=True), navigation=nav, rng=generator(seq),
        t_max=cfg.t_max, done_timeout=cfg.done_timeout, broadcast_interval=cfg.broadcast_interval)
    return out


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--resolver", choices=("mvp", "vo"), default="mvp")
    p.add_argument("--angles", type=float, nargs="+", default=[2.0, 10.0, 45.0, 90.0])
    p.add_argument("--ci95", type=float, default=10.0, help="position CI95 [m]")
    p.add_argument("--velo", type=float, default=1.0, help="velocity CI95 [m/s]")
    p.add_argument("--rpz", type=float, default=50.0)
    p.add_argument("--lookahead", type=float, default=120.0)
    p.add_argument("--tlos", type=float, default=180.0, help="spawn time-to-LoS [s]")
    p.add_argument("--margin", type=float, default=1.05)
    p.add_argument("--speed", type=float, default=10.2889)
    p.add_argument("--dt", type=float, default=0.2)
    p.add_argument("--broadcast-interval", dest="broadcast_interval", type=float, default=1.0)
    p.add_argument("--t-max", dest="t_max", type=float, default=600.0)
    p.add_argument("--done-timeout", dest="done_timeout", type=float, default=10.0)
    p.add_argument("--n", type=int, default=500, help="noise realisations per angle")
    p.add_argument("--jobs", type=int, default=4)
    p.add_argument("--seed", type=int, default=0)
    cfg = p.parse_args()

    seqs = list(spawn(root_seed_sequence(cfg.seed), cfg.n))  # same substreams per angle
    print(f"IPR sweep — resolver={cfg.resolver.upper()}, CI95={cfg.ci95} m, velo={cfg.velo} m/s, "
          f"rpz={cfg.rpz}, lookahead={cfg.lookahead}, tlos={cfg.tlos}, dcpa=0, "
          f"margin={cfg.margin}, {cfg.n} pairs, joblib {cfg.jobs} cores")
    print(f"{'dpsi':>6} {'IPR':>8} {'LoS':>11} {'Median CPA':>14}")
    t0 = time.time()
    for dpsi in cfg.angles:
        outcomes = Parallel(n_jobs=cfg.jobs)(delayed(_one)(dpsi, s, cfg) for s in seqs)
        
        n_los = sum(o.los for o in outcomes)
        median_min_sep = np.median([o.min_sep for o in outcomes])

        print(
            f"{dpsi:6.1f} "
            f"{1 - n_los / cfg.n:8.3f} "
            f"{f'{n_los}/{cfg.n}':>11} "
            f"{median_min_sep:>14.2f} m"
        )
    print(f"(elapsed {time.time() - t0:.1f} s)")


if __name__ == "__main__":
    main()
