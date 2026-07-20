"""Fixed-crossing-angle IPR sweep, parallelised with joblib.

For each crossing angle, place one ``dcpa = 0`` conflict pair and run ``--n`` independent noise
realisations through the encounter loop; report IPR = 1 − LoS / n (denominator = n_pair, since
every pair is a constructed conflict). LoS is measured on the true separation.

Examples
--------
    python scripts/ipr_angle_sweep.py                          # MVP, default sweep
    python scripts/ipr_angle_sweep.py --resolver vo            # same, VO shortest-way-out
    python scripts/ipr_angle_sweep.py --recovery ftr --share-intent   # FTR, intent shared
    python scripts/ipr_angle_sweep.py --resolvers mvp vo --recovery ftr  # side-by-side comparison
    python scripts/ipr_angle_sweep.py --pos-ci95 10 --vel-ci95 1 --angles 2 10 45 90 --n 500 --jobs 4
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
from opencdarr.crr import FTR, PastCPA
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.loop import run_encounter
from opencdarr.performance import M600
from opencdarr.rng import generator, root_seed_sequence, spawn
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState


def _resolver(name: str, margin: float) -> ConflictResolver:
    return {"mvp": MVP, "vo": VO}[name](margin=margin)


def _recovery(name: str, bouncing_guard: bool) -> RecoveryCriterion:
    if name == "pastcpa":
        return PastCPA(bouncing_guard=bouncing_guard)
    if name == "ftr":
        return FTR()
    raise ValueError(f"unknown recovery {name!r}")


def _one(dpsi: float, resolver_name: str, seq, cfg: argparse.Namespace) -> int:
    own = AircraftState(
        id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=cfg.speed,
        pos_ci95=cfg.pos_ci95, vel_ci95=cfg.vel_ci95,
    )
    intr = create_conflict(
        own, intr_id="INT", dpsi=dpsi, dcpa=0.0, tlos=cfg.tlos, rpz=cfg.rpz, side=1)
    nav = GpsNavigation()
    out = run_encounter(
        own, intr, perf=M600, rpz=cfg.rpz, t_lookahead=cfg.lookahead, dt=cfg.dt,
        detector=StateBased(), resolver=_resolver(resolver_name, cfg.margin),
        recovery=_recovery(cfg.recovery, cfg.bouncing_guard), navigation=nav, rng=generator(seq),
        t_max=cfg.t_max, done_timeout=cfg.done_timeout, broadcast_interval=cfg.broadcast_interval,
        share_intent=cfg.share_intent)
    return out


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--resolvers", nargs="+", choices=("mvp", "vo"), default=None,
                   help="compare several resolvers side by side (overrides --resolver)")
    p.add_argument("--resolver", choices=("mvp", "vo"), default="mvp")
    p.add_argument("--recovery", choices=("pastcpa", "ftr"), default="pastcpa")
    p.add_argument("--bouncing-guard", dest="bouncing_guard", action="store_true", default=True)
    p.add_argument("--share-intent", dest="share_intent", action="store_true", default=False,
                   help="let the intruder's desired (nominal) velocity be perceived — FTR's "
                        "second, intent-based criterion (else FTR falls back to its first)")
    p.add_argument("--angles", type=float, nargs="+", default=[2.0, 10.0, 45.0, 90.0])
    p.add_argument("--pos-ci95", dest="pos_ci95", type=float, default=10.0, help="position CI95 [m]")
    p.add_argument("--vel-ci95", dest="vel_ci95", type=float, default=1.0, help="velocity CI95 [m/s]")
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

    resolvers = cfg.resolvers if cfg.resolvers else [cfg.resolver]
    seqs = list(spawn(root_seed_sequence(cfg.seed), cfg.n))  # same substreams per angle/resolver
    print(f"IPR sweep — resolvers={[r.upper() for r in resolvers]}, recovery={cfg.recovery.upper()}"
          f"{' (+intent)' if cfg.share_intent else ''}, pos_ci95={cfg.pos_ci95} m, "
          f"vel_ci95={cfg.vel_ci95} m/s, rpz={cfg.rpz}, lookahead={cfg.lookahead}, tlos={cfg.tlos}, dcpa=0, "
          f"margin={cfg.margin}, {cfg.n} pairs, joblib {cfg.jobs} cores")
    print(f"{'resolver':>9} {'dpsi':>6} {'IPR':>8} {'LoS':>11} {'Median CPA':>14}")
    t0 = time.time()
    for resolver_name in resolvers:
        for dpsi in cfg.angles:
            outcomes = Parallel(n_jobs=cfg.jobs)(
                delayed(_one)(dpsi, resolver_name, s, cfg) for s in seqs)

            n_los = sum(o.los for o in outcomes)
            median_min_sep = np.median([o.min_sep for o in outcomes])

            print(
                f"{resolver_name.upper():>9} "
                f"{dpsi:6.1f} "
                f"{1 - n_los / cfg.n:8.4f} "
                f"{f'{n_los}/{cfg.n}':>11} "
                f"{median_min_sep:>14.2f} m"
            )
    print(f"(elapsed {time.time() - t0:.1f} s)")


if __name__ == "__main__":
    main()
