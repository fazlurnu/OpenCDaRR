"""The MC-vs-IPS validation campaign: one declaration, three scenarios, both estimators.

The claim under test is that plain Monte Carlo and the interacting particle system estimate the
*same* quantity — ``p_los`` per aircraft. Each part runs both backends over the same conditions
from the same seed, reports the ratio, and reports what each backend spent to reach it.

**This module is the single source of the declaration.** The part scripts import their axes from
here, and so does the notebook, because a cached result is keyed on the declaration: the same
config, the same component identities, the same seed and the same backend give the same key. If
the notebook rebuilt the axes itself, one edited number would silently miss every cached cell and
re-simulate the campaign.

The cache directory is **absolute** (repo root), not the relative default. A relative
``.opencdarr_cache`` resolves against the working directory, so a script launched from the repo
root and a notebook launched from ``examples/handbook`` would not share one entry.

**Cost is measured per condition, and it is wall time.** The two backends run one condition at a
time, so ``mc_seconds`` and ``ips_seconds`` are the cost of *that* cell and not one number for the
whole sweep; the IPS figure includes the pilot run that places that cell's ladder. Both come from
the same machine and the same worker count, so the pair is comparable — the absolute values are
not, and do not transfer to another machine. A condition served from the cache is reported as
``cached`` and kept out of the totals, because reading a pickle is not a measurement of a run.
``--no-cache`` is how to time a campaign whose conditions are already stored.
"""

from __future__ import annotations

import argparse
import json
import time
from collections.abc import Iterator
from dataclasses import dataclass
from datetime import UTC, datetime
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
from opencdarr.experiment import IPS, MC, Backend, CacheConfig, Ladder
from opencdarr.parallel import resolve_jobs

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
    p.add_argument("--no-cache", action="store_true",
                   help="simulate every condition and store nothing, so the times are timings "
                        "of runs and not of cache reads")
    return p


# --- one condition at a time ------------------------------------------------------------------


def sweep_of(part: str) -> tuple[str, Sweep]:
    """The one axis a part sweeps, as ``(declaration key, Sweep)``."""
    swept = [(key, axis) for key, axis in PARTS[part].items() if isinstance(axis, Sweep)]
    if len(swept) != 1:
        raise ValueError(f"part {part!r} sweeps {len(swept)} axes; the campaign assumes exactly 1")
    return swept[0]


def conditions_of(part: str) -> Iterator[tuple[Any, dict[str, Any]]]:
    """The part's declaration, one level at a time: ``(level, axes)`` per condition.

    Splitting the sweep is what makes the per-condition cost measurable — a whole-sweep call gives
    one wall time for every condition together. It changes no number: a cell is keyed on its
    *resolved* values, so a one-level sweep and a six-level sweep hit the same cache entry, and
    with more workers than conditions :func:`~opencdarr.experiment.run_experiment` already ran the
    conditions in turn and spent the whole budget inside each one (ADR 0018). Below that worker
    count this trades the fan-out over conditions for the same parallelism inside them.
    """
    key, sweep = sweep_of(part)
    for level in sweep.values:
        yield level, {**PARTS[part], key: Sweep([level], name=sweep.name, build=sweep.build)}


@dataclass(frozen=True)
class Timed:
    """One backend's result for one condition, and the wall time it cost.

    ``simulated`` is false when the cache answered, and then ``seconds`` timed a pickle read. It is
    reported rather than hidden: a fraction of a second written into the results file as the cost
    of a Monte-Carlo batch would be a wrong number, not a fast one.
    """

    row: dict[str, Any]
    seconds: float
    simulated: bool


def _stored() -> set[str]:
    """The cache filenames now — one entry per condition per backend."""
    return {path.name for path in CACHE_DIR.glob("*.pkl")} if CACHE_DIR.is_dir() else set()


def _run(axes: dict[str, Any], backend: Backend, args: argparse.Namespace,
         cache: CacheConfig) -> Timed:
    """Run one condition on one backend, and time it.

    Whether the condition was simulated is read off the cache directory: a computed cell writes its
    one entry, a cached cell writes nothing. (This assumes the campaign is the only writer, which
    is true of a run started from ``run_all.sh``.)
    """
    before = _stored()
    started = time.perf_counter()
    result = run_experiment(axes, methods=STACK, backend=backend, base_config=CFG,
                            seed=args.seed, n_jobs=args.jobs, cache=cache)
    seconds = time.perf_counter() - started
    (row,) = result.records()
    return Timed(row=row, seconds=seconds, simulated=not cache.enabled or bool(_stored() - before))


def duration(seconds: float, simulated: bool = True) -> str:
    """A wall time for a table: seconds under a minute and a half, minutes above it."""
    if not simulated:
        return "cached"
    return f"{seconds:.1f}s" if seconds < 90.0 else f"{seconds / 60:.1f}m"


# --- the campaign -----------------------------------------------------------------------------


def run_part(part: str, args: argparse.Namespace) -> list[dict[str, Any]]:
    """Run one part on both backends, print a row per condition, and write the rows to JSON."""
    cache = CacheConfig(dir=CACHE_DIR, enabled=not args.no_cache)
    workers = resolve_jobs(args.jobs)
    key, sweep = sweep_of(part)
    axis = sweep.name or key

    mc = MC(n_encounters=args.mc_encounters)
    ips = IPS(shells=Ladder(pilot=args.pilot), n_particles=args.particles, reps=args.reps,
              tail=True)

    started_at = datetime.now(UTC)
    started = time.perf_counter()
    print(f"[{part}] Monte Carlo: {args.mc_encounters:,} encounters per condition. "
          f"IPS: {args.particles:,} particles x {args.reps} replications, ladder from a "
          f"{args.pilot:,}-encounter pilot.", flush=True)
    print(f"[{part}] {workers} workers, started {started_at:%Y-%m-%d %H:%M:%S} UTC"
          + ("" if cache.enabled else ", cache off (every condition is simulated)"), flush=True)

    rows: list[dict[str, Any]] = []
    for level, axes in conditions_of(part):
        anchor = _run(axes, mc, args, cache)
        split = _run(axes, ips, args, cache)

        mc_p, ips_p = anchor.row["p_los"], split.row["p_los"]
        factor = tolerance(mc_p) if mc_p > 0 else float("nan")
        ratio = ips_p / mc_p if mc_p > 0 else float("nan")
        timed = anchor.simulated and split.simulated
        rows.append({
            "part": part,
            axis: anchor.row[axis],
            "mc_p_los": mc_p,
            "ips_p_los": ips_p,
            "ratio": ratio,
            "factor": factor,
            "agrees": bool(mc_p > 0 and 1 / factor <= ratio <= factor),
            "mc_mean_los_pairs": anchor.row["mean_los_pairs"],
            "ips_mean_los_pairs": split.row["mean_los_pairs"],
            "median_min_sep": anchor.row["median_min_sep"],
            "n_collapsed": split.row["n_collapsed"],
            # wall clock on `workers` workers of this machine. `timed` is false where the cache
            # answered, and then the two times are reads and the gain means nothing.
            "mc_seconds": anchor.seconds,
            "ips_seconds": split.seconds,
            "gain": (anchor.seconds / split.seconds
                     if timed and split.seconds > 0 else float("nan")),
            "timed": timed,
            "mc_simulated": anchor.simulated,
            "ips_simulated": split.simulated,
        })
        gain = f"{rows[-1]['gain']:.2f}x" if timed and split.seconds > 0 else "--"
        print(f"[{part}] {axis}={level:<6} "
              f"MC {duration(anchor.seconds, anchor.simulated):>8}   "
              f"IPS {duration(split.seconds, split.simulated):>8}   gain {gain:>7}", flush=True)

    print(f"\n{axis:>12}{'MC':>12}{'IPS':>12}{'IPS/MC':>9}{'within':>8}{'collapsed':>11}"
          f"{'agrees':>8}{'MC time':>10}{'IPS time':>10}{'gain':>8}")
    for r in rows:
        ratio = f"{r['ratio']:9.2f}" if r["mc_p_los"] > 0 else f"{'--':>9}"
        within = f"{r['factor']:7.0f}x" if r["mc_p_los"] > 0 else f"{'--':>8}"
        verdict = "yes" if r["agrees"] else ("--" if r["mc_p_los"] <= 0 else "NO")
        gain = f"{r['gain']:7.2f}x" if r["timed"] else f"{'--':>8}"
        print(f"{r[axis]:>12}{r['mc_p_los']:12.3e}{r['ips_p_los']:12.3e}{ratio}{within}"
              f"{r['n_collapsed']:11d}{verdict:>8}"
              f"{duration(r['mc_seconds'], r['mc_simulated']):>10}"
              f"{duration(r['ips_seconds'], r['ips_simulated']):>10}{gain}")

    finished_at = datetime.now(UTC)
    wall = time.perf_counter() - started
    mc_total = sum(r["mc_seconds"] for r in rows if r["mc_simulated"])
    ips_total = sum(r["ips_seconds"] for r in rows if r["ips_simulated"])
    from_cache = sum(1 for r in rows if not (r["mc_simulated"] and r["ips_simulated"]))
    timing = {
        "started": started_at.isoformat(timespec="seconds"),
        "finished": finished_at.isoformat(timespec="seconds"),
        "wall_seconds": wall,
        "mc_seconds": mc_total,
        "ips_seconds": ips_total,
        "gain": mc_total / ips_total if ips_total > 0 else float("nan"),
        "workers": workers,
        "cache": cache.enabled,
        "conditions_from_cache": from_cache,
    }

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    out = RESULTS_DIR / f"{part}.json"
    out.write_text(json.dumps(
        {"part": part, "axis": axis, "settings": vars(args), "timing": timing, "rows": rows},
        indent=1) + "\n")

    measured = [r for r in rows if r["mc_p_los"] > 0]
    disagreed = [r for r in measured if not r["agrees"]]
    collapsed = [r for r in rows if r["n_collapsed"]]
    print(f"\n[{part}] {len(measured)}/{len(rows)} conditions measurable by MC, "
          f"{len(disagreed)} disagree, {len(collapsed)} with a collapsed replication")
    print(f"[{part}] simulation on {workers} workers: MC {mc_total / 60:.1f} min, "
          f"IPS {ips_total / 60:.1f} min"
          + (f" ({from_cache} condition(s) came from the cache)" if from_cache else ""))
    print(f"[{part}] wrote {out}")
    print(f"[{part}] DONE in {wall / 60:.1f} min, at {finished_at:%Y-%m-%d %H:%M:%S} UTC",
          flush=True)
    return rows
