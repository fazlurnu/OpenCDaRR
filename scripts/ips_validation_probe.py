"""Rare-event validation-ladder probe: sweep CNS noise, report P(LoS) with a Wilson CI.

The Phase-8 IPS gate ([[rare-event-validation-ladder]], ADR 0017) needs a regime where brute-force
Monte Carlo measures a *non-zero* loss-of-separation rate — every CDR cell in the handbook MC
showed P(LoS)=0 because the resolver is too good. This script degrades perception (GNSS self-noise)
with the resolver on and reports P(LoS | conflict) as a function of ``pos_ci95``, over the *same*
plain-MC path (:func:`opencdarr.estimator.estimate_ipr`) that IPS is validated against. It is the
reproducible source of the two anchor rungs — pos=40 (correctness) and pos=10 (efficiency).

Each (pos, seed) cell is one seeded ``estimate_ipr`` run of ``--n`` encounters; cells are pooled
per ``pos`` over ``--reps`` seeds and joblib-parallelised. ``vel_ci95 = pos_ci95 * --vel-ratio``.

Examples
--------
    python scripts/ips_validation_probe.py                         # the ladder, n=1000, 1 seed
    python scripts/ips_validation_probe.py --pos 40 --n 3000 --reps 3 --jobs 3   # anchor, pooled
    python scripts/ips_validation_probe.py --pos 10 --n 10000 --reps 3 --jobs 3  # rare rung
    python scripts/ips_validation_probe.py --no-resolver --pos 10 --n 400        # sanity (P~1)
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import numpy as np
from joblib import Parallel, delayed

from opencdarr.cd import StateBased
from opencdarr.cns import GnssNavigation
from opencdarr.config import (
    Config,
    ConflictConfig,
    MethodsConfig,
    ScenarioConfig,
    SimulationConfig,
)
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.estimator import estimate_ipr
from opencdarr.performance import M600


@dataclass(frozen=True)
class Cell:
    """One (pos, seed) result: the LoS count over its encounters.

    The denominator is the *encounter* count, matching ``estimate_ipr`` and IPS's all-N denominator
    (``opencdarr.estimator.IPRResult``) — not the detected-conflict count, which moves with the
    resolver."""

    pos: float
    n_encounters: int
    n_los: int


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float, float]:
    """Point estimate and Wilson-score 95% CI for a binomial rate ``k/n``."""
    if n == 0:
        return (float("nan"), 0.0, 1.0)
    p = k / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (p, max(0.0, centre - half), min(1.0, centre + half))


def _config(pos: float, seed: int, cfg: argparse.Namespace) -> Config:
    return Config(
        seed=seed,
        n_encounters=cfg.n,
        scenario=ScenarioConfig(
            aircraft_type="M600", speed=cfg.speed, dcpa_max=cfg.dcpa_max, tlos=cfg.tlos,
            pos_ci95=pos, vel_ci95=pos * cfg.vel_ratio,
        ),
        conflict=ConflictConfig(rpz=cfg.rpz, t_lookahead=cfg.lookahead),
        methods=MethodsConfig(detection="statebased", resolution="mvp", recovery="pastcpa",
                              margin=cfg.margin, bouncing_guard=True),
        simulation=SimulationConfig(dt=cfg.dt, t_max=cfg.t_max, done_timeout=cfg.done_timeout),
    )


def _cell(pos: float, seed: int, cfg: argparse.Namespace) -> Cell:
    """One seeded MC cell over the plain-MC estimator — the path IPS is validated against."""
    resolver = None if cfg.no_resolver else MVP(margin=cfg.margin)
    recovery = None if cfg.no_resolver else PastCPA(bouncing_guard=True)
    r = estimate_ipr(
        _config(pos, seed, cfg), M600, StateBased(), resolver, recovery,
        navigation=GnssNavigation(),
    )
    return Cell(pos=pos, n_encounters=r.n_encounters, n_los=r.n_los)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--pos", type=float, nargs="+", default=[10, 20, 30, 40, 60, 80],
                   help="pos_ci95 values [m] to sweep")
    p.add_argument("--n", type=int, default=1000, help="encounters per (pos, seed) cell")
    p.add_argument("--reps", type=int, default=1, help="seeds pooled per pos (seed, seed+1, ...)")
    p.add_argument("--seed", type=int, default=0, help="base seed")
    p.add_argument("--jobs", type=int, default=1, help="joblib parallel cells")
    p.add_argument("--vel-ratio", dest="vel_ratio", type=float, default=0.1,
                   help="vel_ci95 = pos_ci95 * this")
    p.add_argument("--no-resolver", dest="no_resolver", action="store_true",
                   help="baseline: no CDR (P(LoS) should be ~1)")
    p.add_argument("--speed", type=float, default=10.2889)
    p.add_argument("--dcpa-max", dest="dcpa_max", type=float, default=0.0)
    p.add_argument("--tlos", type=float, default=90.0)
    p.add_argument("--rpz", type=float, default=50.0)
    p.add_argument("--lookahead", type=float, default=120.0)
    p.add_argument("--margin", type=float, default=1.05)
    p.add_argument("--dt", type=float, default=0.5)
    p.add_argument("--t-max", dest="t_max", type=float, default=250.0)
    p.add_argument("--done-timeout", dest="done_timeout", type=float, default=10.0)
    cfg = p.parse_args()

    cells = [(pos, cfg.seed + r) for pos in cfg.pos for r in range(cfg.reps)]
    print(f"probe: {len(cfg.pos)} pos x {cfg.reps} seeds x n={cfg.n}  "
          f"resolver={'off' if cfg.no_resolver else f'MVP(margin={cfg.margin})+PastCPA'}, "
          f"vel_ci95=pos*{cfg.vel_ratio}, rpz={cfg.rpz}, dt={cfg.dt}, jobs={cfg.jobs}")
    t0 = time.perf_counter()
    results: list[Cell] = Parallel(n_jobs=cfg.jobs)(
        delayed(_cell)(pos, seed, cfg) for pos, seed in cells
    )
    print(f"{'pos_ci95':>9} {'LoS':>10} {'P(LoS)':>10} {'95% CI':>22}")
    for pos in cfg.pos:
        n = sum(c.n_encounters for c in results if c.pos == pos)
        k = sum(c.n_los for c in results if c.pos == pos)
        prob, lo, hi = wilson(k, n)
        print(f"{pos:9.0f} {f'{k}/{n}':>10} {prob:10.5f}   [{lo:.5f}, {hi:.5f}]")
    print(f"({time.perf_counter() - t0:.0f}s)")


if __name__ == "__main__":
    main()
