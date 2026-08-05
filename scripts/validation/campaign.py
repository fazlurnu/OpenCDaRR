"""The MC-vs-IPS validation campaign: one declaration, three scenarios, both estimators.

The claim under test is that plain Monte Carlo and the interacting particle system estimate the
*same* quantity — ``p_los`` per aircraft. Each part runs both backends over the same conditions
from the same seed, and reports the ratio.

**This module is the single source of the declaration.** The part scripts import their axes from
here, and so does the notebook, because a cached result is keyed on the declaration: the same
config, the same component identities, the same seed and the same backend give the same key. If
the notebook rebuilt the axes itself, one edited number would silently miss every cached cell and
re-simulate the campaign.

The cache directory is **absolute** (repo root), not the relative default. A relative
``.opencdarr_cache`` resolves against the working directory, so a script launched from the repo
root and a notebook launched from ``examples/handbook`` would not share one entry.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

from opencdarr import (
    M600,
    MVP,
    CrossingRing,
    Fixed,
    GnssNavigation,
    Methods,
    PairwiseEncounter,
    PastCPA,
    RandomTraffic,
    StateBased,
    Sweep,
    run_experiment,
)
from opencdarr.config import (
    Config,
    ConflictConfig,
    MethodsConfig,
    ScenarioConfig,
    SimulationConfig,
)
from opencdarr.experiment import IPS, MC, CacheConfig, Ladder

REPO_ROOT = Path(__file__).resolve().parents[2]
CACHE_DIR = REPO_ROOT / ".opencdarr_cache"
RESULTS_DIR = REPO_ROOT / "results" / "validation"

# The CNS condition of the whole campaign: a 10 m / 1 m/s GNSS fix on every aircraft.
POS_CI95, VEL_CI95 = 10.0, 1.0

CFG = Config(
    seed=0,
    n_encounters=0,          # the backend sets this
    scenario=ScenarioConfig(aircraft_type="M600", speed=10.0, dcpa_max=50.0, tlos=60.0,
                            pos_ci95=POS_CI95, vel_ci95=VEL_CI95),
    conflict=ConflictConfig(rpz=50.0, t_lookahead=30.0),
    methods=MethodsConfig(detection="statebased", resolution="mvp", recovery="pastcpa",
                          margin=1.05, bouncing_guard=True),  # only run_one_experiment reads this
    simulation=SimulationConfig(dt=0.5, t_max=600.0, done_timeout=10.0),
)

# Held across every part, so the scenario is the only thing that changes between them.
STACK = Methods(detector=StateBased(), resolver=MVP(1.05),
                recovery=PastCPA(bouncing_guard=True), navigation=GnssNavigation(), perf=M600)

CNS = {"pos_ci95": Fixed(POS_CI95), "vel_ci95": Fixed(VEL_CI95)}

# --- the three parts, as declarations ---------------------------------------------------------

PAIRWISE_ANGLES = (2.0, 5.0, 10.0, 45.0, 90.0, 180.0)
FLEET_SIZES = (4, 6, 8)

PARTS: dict[str, dict[str, Any]] = {
    "pairwise": {
        **CNS,
        "scenario": Sweep(PAIRWISE_ANGLES, name="dpsi",
                          build=lambda a: PairwiseEncounter(dpsi=float(a))),
    },
    "ring": {
        **CNS,
        "scenario": Sweep(FLEET_SIZES, name="n_aircraft",
                          build=lambda n: CrossingRing(int(n), radius=500.0)),
    },
    "random_traffic": {
        **CNS,
        "scenario": Sweep(FLEET_SIZES, name="n_aircraft",
                          build=lambda n: RandomTraffic(int(n), r_inner=1000.0, r_outer=1200.0)),
    },
}


def tolerance(p: float) -> float:
    """The factor the two estimators must agree within at a rate of ``p``.

    Two, except where the event is rare enough that the Monte-Carlo anchor is itself coarse: at
    1e-4 and below even a large batch counts only a handful of events, so five is the honest bar.
    """
    return 5.0 if p <= 1e-4 else 2.0


def parser(part: str) -> argparse.ArgumentParser:
    """The CLI every part script shares. ``--jobs`` is the one to set on a bigger machine."""
    p = argparse.ArgumentParser(description=f"MC-vs-IPS validation, part: {part}")
    p.add_argument("--jobs", type=int, default=-1,
                   help="worker processes; -1 is every core (e.g. --jobs 100)")
    p.add_argument("--mc-encounters", type=int, default=50_000,
                   help="encounters per condition for the Monte-Carlo anchor")
    p.add_argument("--particles", type=int, default=2_000, help="IPS particles per shell")
    p.add_argument("--reps", type=int, default=10, help="independent IPS replications")
    p.add_argument("--pilot", type=int, default=2_000,
                   help="pilot encounters the per-condition shell ladder is placed from")
    p.add_argument("--seed", type=int, default=0, help="the reproducibility root")
    return p


def run_part(part: str, args: argparse.Namespace) -> list[dict[str, Any]]:
    """Run one part on both backends, print a row per condition, and write the rows to JSON."""
    axes = PARTS[part]
    cache = CacheConfig(dir=CACHE_DIR)
    started = time.perf_counter()

    print(f"[{part}] Monte Carlo: {args.mc_encounters:,} encounters per condition, "
          f"{args.jobs} workers", flush=True)
    anchor = run_experiment(axes, methods=STACK, backend=MC(n_encounters=args.mc_encounters),
                            base_config=CFG, seed=args.seed, n_jobs=args.jobs, cache=cache)

    print(f"[{part}] IPS: {args.particles:,} particles x {args.reps} replications, "
          f"ladder from a {args.pilot:,}-encounter pilot", flush=True)
    split = run_experiment(axes, methods=STACK,
                           backend=IPS(shells=Ladder(pilot=args.pilot),
                                       n_particles=args.particles, reps=args.reps, tail=True),
                           base_config=CFG, seed=args.seed, n_jobs=args.jobs, cache=cache)

    axis = split.axes[0]
    rows: list[dict[str, Any]] = []
    for mc_row, ips_row in zip(anchor.records(), split.records(), strict=True):
        mc_p, ips_p = mc_row["p_los"], ips_row["p_los"]
        factor = tolerance(mc_p) if mc_p > 0 else float("nan")
        ratio = ips_p / mc_p if mc_p > 0 else float("nan")
        rows.append({
            "part": part,
            axis: mc_row[axis],
            "mc_p_los": mc_p,
            "ips_p_los": ips_p,
            "ratio": ratio,
            "factor": factor,
            "agrees": bool(mc_p > 0 and 1 / factor <= ratio <= factor),
            "mc_mean_los_pairs": mc_row["mean_los_pairs"],
            "ips_mean_los_pairs": ips_row["mean_los_pairs"],
            "median_min_sep": mc_row["median_min_sep"],
            "n_collapsed": ips_row["n_collapsed"],
        })

    print(f"\n{axis:>12}{'MC':>12}{'IPS':>12}{'IPS/MC':>9}{'within':>8}{'collapsed':>11}   agrees")
    for r in rows:
        ratio = f"{r['ratio']:9.2f}" if r["mc_p_los"] > 0 else f"{'--':>9}"
        within = f"{r['factor']:7.0f}x" if r["mc_p_los"] > 0 else f"{'--':>8}"
        verdict = "yes" if r["agrees"] else ("--" if r["mc_p_los"] <= 0 else "NO")
        print(f"{r[axis]:>12}{r['mc_p_los']:12.3e}{r['ips_p_los']:12.3e}{ratio}{within}"
              f"{r['n_collapsed']:11d}   {verdict}")

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    out = RESULTS_DIR / f"{part}.json"
    out.write_text(json.dumps(
        {"part": part, "axis": axis, "settings": vars(args), "rows": rows}, indent=1) + "\n")

    measured = [r for r in rows if r["mc_p_los"] > 0]
    disagreed = [r for r in measured if not r["agrees"]]
    collapsed = [r for r in rows if r["n_collapsed"]]
    print(f"\n[{part}] {len(measured)}/{len(rows)} conditions measurable by MC, "
          f"{len(disagreed)} disagree, {len(collapsed)} with a collapsed replication")
    print(f"[{part}] wrote {out}")
    print(f"[{part}] DONE in {(time.perf_counter() - started) / 60:.1f} min", flush=True)
    return rows
